/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/IntakeSubsystem.h"
#include "Robot.h"

IntakeSubsystem::IntakeSubsystem() : Subsystem("IntakeSubsystem") {
  intakeWheelsTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kINTAKE_WHEELS_ID);
  intakeWristTalonLeft = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kINTAKE_LEFT_WRIST_ID);
  intakeWristTalonRight = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->KINTAKE_RIGHT_WRIST_ID);
  intakeSlapperTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kINTAKE_SLAPPER_ID);
  
  intakeSlapperTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute);
  intakeWristTalonLeft->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute);
}

void IntakeSubsystem::InitDefaultCommand() {
  
}

void IntakeSubsystem::SetIntakeWheelSpeed(double speed) {
  intakeWheelsTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::SetWristAngle(double angle) {
  angle = ConvertAngleToTicksWrist(angle);
  intakeWristTalonLeft->Set(ctre::phoenix::motorcontrol::ControlMode::Position, angle);
}

void IntakeSubsystem::SetSlapperAngle(double ticks) {
  intakeSlapperTalon->Set(ctre::phoenix::motorcontrol::ControlMode::Position, ticks);
}

int IntakeSubsystem::ConvertAngleToTicksWrist(double angle) {
  int ticks = angle / 360.0 * Robot::robotMap->kTICKS_PER_REV_OF_ENCODER * Robot::robotMap->kWRIST_GEAR_RATIO;
  return ticks;
}

int IntakeSubsystem::GetSlapperError() {
  return intakeSlapperTalon->GetClosedLoopError(0);
}