/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

class ElevatorSubsystem : public frc::Subsystem {
 private:
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> elevatorMotorLeader;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> elevatorMotorFollower01;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> elevatorMotorFollower02;
 public:
  ElevatorSubsystem();
  void InitDefaultCommand() override;
  void GoToHeight(double inputHeight);
  int ConvertHeightToTicks(double inputHeight);
  void RunElevatorMotor(double speed);
};
