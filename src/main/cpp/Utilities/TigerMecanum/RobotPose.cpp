/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Utilities/TigerMecanum/RobotPose.h"


RobotPose::RobotPose() {
    x = 0;
    y = 0;
    heading = 0;
}

void RobotPose::SetX(double xIn) {
    x = xIn;
}

void RobotPose::SetY(double yIn) {
    y = yIn;
}

void RobotPose::SetHeading(double headingIn) {
    heading = headingIn;
}

double RobotPose::GetX() {
    return x;
}

double RobotPose::GetY() {
    return y;
}

double RobotPose::GetHeading() {
    return heading;
}

void RobotPose::InitSendable(frc::SendableBuilder& builder) {
    builder.SetSmartDashboardType("RobotPose");
    builder.AddDoubleProperty("x", [=]() { return GetX(); }, [=](double value) {SetX(value);});
    builder.AddDoubleProperty("y",  [=]() { return GetY(); }, [=](double value) {SetY(value);});
    builder.AddDoubleProperty("heading",  [=]() { return GetHeading(); }, [=](double value) {SetHeading(value);});
}
