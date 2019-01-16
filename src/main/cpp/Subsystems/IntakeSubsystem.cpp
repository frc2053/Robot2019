/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/IntakeSubsystem.h"
#include "RobotMap.h"

IntakeSubsystem::IntakeSubsystem() : Subsystem("IntakeSubsystem") {
  intakeWheelsTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kINTAKE_WHEELS_ID);
  intakeWristTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kINTAKE_ACTUATOR_ID);
  intakeFlapperLeftTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kINTAKE_FLAPPER_LEFT_ID);
  intakeFlapperRightTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kINTAKE_FLAPPER_RIGHT_ID);
  
  intakeFlapperRightTalon->Follow(*intakeFlapperLeftTalon.get());

  intakeFlapperLeftTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
  intakeWristTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
}

void IntakeSubsystem::InitDefaultCommand() {
  
}

void IntakeSubsystem::SetIntakeWheelSpeed(double speed) {
  intakeWheelsTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::SetWristAngle(double angle) {
  angle = ConvertAngleToTicksWrist(angle);
  intakeWristTalon->Set(ctre::phoenix::motorcontrol::ControlMode::Position, angle);
}

void IntakeSubsystem::SetFlapperAngle(double angle) {
  angle = ConvertAngleToTicksFlapper(angle);
  intakeFlapperLeftTalon->Set(ctre::phoenix::motorcontrol::ControlMode::Position, angle);
}

int IntakeSubsystem::ConvertAngleToTicksWrist(double angle) {
  int ticks = angle / 360.0 * kTICKS_PER_REV_OF_ENCODER * kWRIST_GEAR_RATIO;
  return ticks;
}

int IntakeSubsystem::GetFlapperAngleError() {
  return intakeFlapperLeftTalon->GetClosedLoopError(0);
}
int IntakeSubsystem::GetWristAngleError() {
  return intakeWristTalon->GetClosedLoopError(0);
}

int IntakeSubsystem::ConvertAngleToTicksFlapper(double angle) {
  int ticks = angle / 360.0 * kTICKS_PER_REV_OF_ENCODER * kFLAPPER_GEAR_RATIO;
}