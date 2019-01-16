/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/ElevatorSubsystem.h"
#include "RobotMap.h"

ElevatorSubsystem::ElevatorSubsystem() : Subsystem("ElevatorSubsystem") {
  elevatorMotorLeader = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kELEVATOR_LEADER_ID);
  elevatorMotorFollower01 = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kELEVATOR_FOLLOWERONE_ID);
  elevatorMotorFollower02 = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kELEVATOR_FOLLOWERTWO_ID);

  elevatorMotorFollower01->Follow(*elevatorMotorLeader.get());
  elevatorMotorFollower02->Follow(*elevatorMotorLeader.get());

  elevatorMotorLeader->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::CTRE_MagEncoder_Relative);
  elevatorMotorLeader->SetSelectedSensorPosition(0);
  elevatorMotorLeader->ConfigAllowableClosedloopError(0, 50, 0);

  elevatorMotorLeader->Config_kF(0, kELEVATOR_F);
  elevatorMotorLeader->Config_kP(0, kELEVATOR_P);
  elevatorMotorLeader->Config_kI(0, kELEVATOR_I);
  elevatorMotorLeader->Config_kD(0, kELEVATOR_D);
}

void ElevatorSubsystem::InitDefaultCommand() {

}

int ElevatorSubsystem::ConvertHeightToTicks(double inputHeight) {
  int ticks = 0;
  ticks = inputHeight * kELEVATORTICKS_PER_INCH;
  return ticks;
}

void ElevatorSubsystem::RunElevatorMotor(double speed) {
  elevatorMotorLeader->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void ElevatorSubsystem::GoToHeight(double height) {
  elevatorMotorLeader->Set(ctre::phoenix::motorcontrol::ControlMode::Position, ConvertHeightToTicks(height));
}

