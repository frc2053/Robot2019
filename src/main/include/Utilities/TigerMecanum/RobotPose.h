/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/smartdashboard/SendableBase.h>
#include <frc/smartdashboard/SendableBuilder.h>

class RobotPose : frc::SendableBase {
 public:
  RobotPose();
  void InitSendable(frc::SendableBuilder& builder);
  void SetX(double xIn);
  void SetY(double yIn);
  void SetHeading(double heading);
  double GetX();
  double GetY();
  double GetHeading();
 private:
  double x;
  double y;
  double heading;
};
