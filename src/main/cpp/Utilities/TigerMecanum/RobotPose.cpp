#include "Utilities/TigerMecanum/RobotPose.h"
#include <iostream>

RobotPose::RobotPose() : SendableBase(false) {
    SetName("RobotPose");
    x = 0;
    y = 0;
    heading = 0;
}

void RobotPose::SetX(double newX) {
    x = newX;
}

void RobotPose::SetY(double newY) {
    y = newY;
}

void RobotPose::SetHeading(double newHeading) {
    heading = newHeading;
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
    builder.AddDoubleProperty("x", [=]() { return GetX();}, [=](double value){SetX(value);});
    builder.AddDoubleProperty("y", [=]() { return GetY();}, [=](double value){SetY(value);});
    builder.AddDoubleProperty("heading", [=]() { return GetHeading();}, [=](double value){SetHeading(value);});
}