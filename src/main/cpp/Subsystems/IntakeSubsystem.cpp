/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/IntakeSubsystem.h"
#include "Robot.h"
#include <iostream>

IntakeSubsystem::IntakeSubsystem() : Subsystem("IntakeSubsystem") {
  intakeWheelsTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kINTAKE_WHEELS_ID);
  intakeWristTalonLeft = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kINTAKE_LEFT_WRIST_ID);
  intakeWristTalonRight = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->KINTAKE_RIGHT_WRIST_ID);
  intakeSlapperTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kINTAKE_SLAPPER_ID);

  intakeWheelsTalon->ConfigFactoryDefault();
  intakeWristTalonLeft->ConfigFactoryDefault();
  intakeWristTalonRight->ConfigFactoryDefault();
  intakeSlapperTalon->ConfigFactoryDefault();
  
  intakeSlapperTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute);
  intakeWristTalonRight->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute);


  intakeSlapperTalon->Config_kF(0, Robot::robotMap->kSLAPPER_F);
  intakeSlapperTalon->Config_kP(0, Robot::robotMap->kSLAPPER_P);
  intakeSlapperTalon->Config_kI(0, Robot::robotMap->kSLAPPER_I);
  intakeSlapperTalon->Config_kD(0, Robot::robotMap->kSLAPPER_D);

  intakeWristTalonRight->Config_kF(0, Robot::robotMap->kSLAPPER_F);
  intakeWristTalonRight->Config_kP(0, Robot::robotMap->kSLAPPER_P);
  intakeWristTalonRight->Config_kI(0, Robot::robotMap->kSLAPPER_I);
  intakeWristTalonRight->Config_kD(0, Robot::robotMap->kSLAPPER_D);

  intakeWristTalonLeft->Follow(*intakeWristTalonRight.get());

}

void IntakeSubsystem::InitDefaultCommand() {
  
}

void IntakeSubsystem::SetIntakeWheelSpeed(double speed) {
  intakeWheelsTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::SetWristAngle(double angle) {
  int ticks;
  ticks = ConvertAngleToTicksWrist(angle);
  //std::cout << "===> This many ticks: " << ticks << std::endl;
  intakeWristTalonRight->Set(ctre::phoenix::motorcontrol::ControlMode::Position, ticks);
}

void IntakeSubsystem::SetWristTicks(int ticks) {
  std::cout << "===> This many ticks: " << ticks << std::endl;
  intakeWristTalonRight->Set(ctre::phoenix::motorcontrol::ControlMode::Position, ticks);
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

int IntakeSubsystem::GetWristError() {
  return intakeWristTalonRight->GetClosedLoopError(0);
}

double IntakeSubsystem::GetCurrent(){
  return intakeWheelsTalon->GetOutputCurrent();
}

