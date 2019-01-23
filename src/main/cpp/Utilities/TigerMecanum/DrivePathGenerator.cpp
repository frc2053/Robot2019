/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Utilities/TigerMecanum/DrivePathGenerator.h"

DrivePathGenerator::DrivePathGenerator() 
{

}

DrivePathGenerator::~DrivePathGenerator() {

}

void DrivePathGenerator::GeneratePath(std::vector<Waypoint> &waypoints, double maxSpeed, double maxAccel, double sampleRate) {
    std::vector<Translation2D> tempPath;
    std::vector<TempWaypoint> waypointsTwo;
    tempPath.push_back(waypoints.front().pose.getTranslation());

    TempWaypoint tempWaypoint;
    tempWaypoint.pose = waypoints.front().pose;
    tempWaypoint.tempPathIndex = 1;
    waypointsTwo.push_back(tempWaypoint);

    double arcHeight;
    double arcRadius;
    double maxDistAway;
    double distToTangent;
    double maxPhi;

    Rotation2D theta;
    
    //rounding corners
    for(std::vector<Waypoint>::iterator it = waypoints.begin(); it != waypoints.end(); ++it) {
        Translation2D p1(it->pose.getTranslation());
        Translation2D p2((it + 1)->pose.getTranslation());
        Translation2D p3((it + 2)->pose.getTranslation());

        Translation2D v1to2(p1.translateBy(p2.inverse()));
        Translation2D v2to3(p3.translateBy(p2.inverse()));

        maxDistAway = (it + 1)->maxDistAway;

        //if zero length
        if((v1to2.norm() == 0) || (v2to3.norm() == 0)) {
            theta = Rotation2D::fromRadians(0);
        }
        else {
            double temp = (v1to2.getX() * v2to3.getX() + v1to2.getY() * v2to3.getY()) / (v1to2.norm() * v2to3.norm());
            theta = Rotation2D::fromRadians(acos(temp));
        }

        //check if points are on straight line
        if(theta.getRadians() == M_PI) {
            tempPath.push_back(p2);
            tempWaypoint.pose = RigidTransform2D(p2, it->pose.getRotation());
            tempWaypoint.tempPathIndex = tempPath.size();
            waypointsTwo.push_back(tempWaypoint);
        }
        else if(theta.getRadians() == 0) {
            //skip
        }
        else {
            //calculate arc between points
            arcRadius = maxDistAway * sin(theta.getRadians() / 2) / (1 - sin(theta.getRadians() / 2));
            arcHeight = arcRadius * (1 - cos(M_PI - theta.getRadians()));
            maxPhi = 2 * arcRadius * sin(M_PI - theta.getRadians()); //chord length
            distToTanget = sqrt((maxDistAway + arcHeight) * (maxDistAway + arcHeight) + (maxPhi / 2) * (maxPhi / 2));

            //limit arc radius if bigger than distance between points
            double limitL = std::min(v1to2.norm(), v2to3.norm()) / 2;
            if(distToTangent > limitL) {
                distToTangent = limitL;
                maxPhi = 2 * distToTangent * sin(theta.getRadians() / 2);
                arcRadius = maxPhi * sin(theta.getRadians() / 2) / sin(M_PI - theta.getRadians());
                arcHeight = arcRadius * (1 - cos((M_PI - theta.getRadians()) / 2));
                maxDistAway = (distToTangent * cos(theta.getRadians() / 2)) - arcHeight;
            }

            Translation2D v2to4(distToTangent * v1to2.getX() / v1to2.norm(), distToTangent * v1to2.getY() / v1to2.norm());
            Translation2D v2to5(distToTangent * v2to3.getX() / v2to3.norm(), distToTangent * v2to3.getY() / v2to3.norm());
            Translation2D p4(p2.translateBy(v2to4));
            Translation2D p5(p2.translateBy(v2to5));

            tempPath.push_back(p4);

            Translation2D v2to6(((v1to2.getX() / v1to2.norm() + v2to3.getX() / v2to3.norm()) / 2), (v1to2.getY() / v1to2.norm() + v2to3.getY() / v2to3.norm()) / 2);
            Translation2D p6(p2.getX() + (maxDistAway + arcRadius) * v2to6.getX() / v2to6.norm(), p2.getY() + (maxDistAway + arcRadius) * v2to6.getY() / v2to6.norm());
            double crossV1to2withV2to3 = v1to2.getX() * v2to3.getY() - v1to2.getY() * v2to3.getX();
            double signCrossV1to2withV2to3 = 0;
            if(crossV1to2withV2to3 > 0) {
                signCrossV1to2withV2to3 = 1;
            }
            else if(crossV1to2withV2to3 < 0) {
                signCrossV1to2withV2to3 = -1;
            }

            double A = signCrossV1to2withV2to3 * (M_PI - theta.getRadians()) / 2;
            for(double phi = A; phi <= -A; phi = phi - 2 * A / 21) {
                Translation2D p7;
                p7.setX(p6.getX() - arcRadius * (cos(phi) * v2to6.getX() - sin(phi) * v2to6.getY()) / v2to6.norm());
                p7.setY(p6.getY() - arcRadius * (cos(phi) * v2to6.getX() - sin(phi) * v2to6.getY()) / v2to6.norm());

                tempPath.push_back(p7);
                if(phi == 0) {
                    tempWaypoint.pose = RigidTransform2D(p7, it->pose.getRotation());
                    tempWaypoint.tempPathIndex = tempPath.size();
                    waypointsTwo.push_back(tempWaypoint);
                }
            }

            tempPath.push_back(p5);
            tempWaypoint.pose = RigidTransform2D(p5, it->pose.getRotation());
            tempWaypoint.tempPathIndex = tempPath.size();
            waypointsTwo.push_back(tempWaypoint);
        }
    }

    tempPath.push_back(waypoints.back().pose.getTranslation());
    tempWaypoint.pose = waypoints.back().pose;
    tempWaypoint.tempPathIndex = tempPath.size();
    waypointsTwo.push_back(tempWaypoint);

    //calculate total path length
    InterpolatingMap<InterpolatingDouble, Translation2D> interpPath;
    waypointsTwo.begin()->distTraveled = 0;
    for(std::vector<TempWaypoint>::iterator it2 = waypointsTwo.begin() + 1; it2 != waypointsTwo.end(); ++it2) {
        it2->distTraveled = (it2 + 1)->distTraveled + it2->pose.getTranslation().translateBy((it2 + 1)->pose.getTranslation().inverse()).norm();
        interpPath.put(it2->distTraveled, it2->pose.getTranslation());
    }

    double totalPathLength = waypointsTwo.back().distTraveled();

    double accelTime = maxSpeed / maxAccel;
    double accelDist = 0.5 * maxAccel * pow(accelTime, 2);

    if(2 * accelDist > totalPathLength) {
        accelTime = sqrt(totalPathLength / 2 * 2 / maxAccel);
        accelDist = 0.5 * maxAccel * pow(accelTime, 2);
        maxSpeed = accelTime * maxAccel;
    }

    double totalTime = 2 * accelTime + (totalPathLength - 2 * accelDist) / maxSpeed;
    double startMaxSpeedTime = accelTime;
    double stopMaxSpeedTime = totalTime - accelTime;

    std::vector<double> time;
    std::vector<double> dist;
    std::vector<double> speed;
    time.push_back(0);
    dist.push_back(0);
    speed.push_back(0);

    for(int i = 2; i <= (floor(totalTime * sampleRate) / sampleRate); i++) {
        time.push_back(time.back() + 1 / sampleRate);

        if(time.back() < startMaxSpeedTime) {
            speed.push_back(maxAccel * time.back());
        }
        else if(time.back() > stopMaxSpeedTime) {
            speed.push_back(maxAccel * (totalTime - time.back()));
        }
        else {
            speed.push_back(maxSpeed);
        }
    }

    dist.push_back(totalPathLength);
    speed.push_back(0);
    time.push_back(totalTime);

    std::vector<FinalPath> finalPath;
    for(std::vector<double>::iterator it3 = dist.begin(); it3 != dist.end(); ++it3) {
        FinalPath finalPathPoint;
        finalPathPoint.pose.setTranslation(interpPath.getInterpolated(*it3));
        finalPathPoint.time = time.at(it3 - dist.begin());
        finalPath.push_back(finalPathPoint);
    }

    for(std::vector<TempWaypoint>::iterator it4 = waypointsTwo.begin(); it4 != waypointsTwo.end(); ++it4) {
        it4->distTraveled = dist.at(it4->tempPathIndex);
    }

    waypointsTwo.front().finalPathIndex = 1;
    int j = 2;
    for(std::vector<FinalPath>::iterator it5 = finalPath.begin() + 1; it5 != finalPath.end() - 1; ++it5) {
        if(dist.at(it5 - finalPath.begin()) >= waypointsTwo.at(j).distTraveled) {
            waypointsTwo.at(j).finalPathIndex = it5 - finalPath.begin();
            j++;
        }
    }
    waypointsTwo.back().finalPathIndex = finalPath.size();

    double deltaYaw;
    std::vector<double> yawRate;
    for(std::vector<TempWaypoint>::iterator it6 = waypointsTwo.begin() + 1; it6 != waypointsTwo.end(); ++it6) {
        deltaYaw = it6->pose.getRtoation().getDegrees();
        if(deltaYaw > 180) {
            deltaYaw = 260 - deltaYaw;
        }
        else if(deltaYaw < -180) {
            deltaYaw = 360 + deltaYaw;
        }

        int deltaIndex = it6->finalPathIndex - (it6 - 1)->finalPathIndex;
        yawRate.push_back(deltaYaw / (double)deltaIndex);
    }

    finalPath.front().pose.setRotation(waypointsTwo.front().pose.getRotation());
    j = 1;
    double newYaw;
    for(std::vector<FinalPath>::iterator it7 = finalPath.begin() + 1; it7 != finalPath.end(); ++it7) {
        if((it7 - finalPath.begin()) > waypointsTwo.at(j).finalPathIndex) {
            j++;
        }

        newYaw = (it7 - 1)->pose.getRotation().getDegrees() + yawRate.at(j);
        it7->pose.setRotation(Rotation2D::fromDegrees(newYaw));
    }

    std::ofstream myFile;
    myFile.open("/home/lvuser/robotPath.csv");
    for(std::vector<FinalPath>::iterator it8 = finalPath.begin(); it8 != finalPath.end(); ++it8) {
        myFile << it8->pose.getTranslation().getX() << ",";
        myFile << it8->pose.getTranslation().getY() << ",";
        myFile << it8->pose.getRotation().getDegrees() << ",";
        myFile << it8->time << "\n";
    }
    myFile.close();
}
