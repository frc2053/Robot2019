/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>

class IntakeSubsystem : public frc::Subsystem {
 private:
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> intakeWheelsTalon;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> intakeWristTalon;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> intakeFlapperLeftTalon;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> intakeFlapperRightTalon;
 public:
  IntakeSubsystem();
  void InitDefaultCommand() override;
  void SetIntakeWheelSpeed(double speed);
  void SetWristAngle(double angle);
  void SetFlapperAngle(double angle);
  int ConvertAngleToTicksWrist(double angle);
  int ConvertAngleToTicksFlapper(double angle);
  int GetWristAngleError();
  int GetFlapperAngleError();
};
