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
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> intakeWristTalonLeft;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> intakeWristTalonRight;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> intakeSlapperTalon;
  double arbFF;
 public:
  IntakeSubsystem();
  void InitDefaultCommand() override;
  void SetIntakeWheelSpeed(double speed);
  void SetWristAngle(double angle);
  void SetWristTicks(int ticks);
  void SetSlapperAngle(double angle);
  int ConvertAngleToTicksWrist(double angle);
  int ConvertAngleToTicksSlapper(double angle);
  int GetSlapperError();
  int GetWristError();
  double GetCurrent();
  virtual void Periodic();
};
