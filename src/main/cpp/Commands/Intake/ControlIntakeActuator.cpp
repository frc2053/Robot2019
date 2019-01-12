/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Intake/ControlIntakeActuator.h"
#include "RobotMap.h"
#include "Robot.h"

ControlIntakeActuator::ControlIntakeActuator(double intakeAngle) {
  Requires(Robot::intakeSubsystem.get());
  targetAngle = intakeAngle;
  isDone = false;
}

// Called just before this Command runs the first time
void ControlIntakeActuator::Initialize() {
  isDone = false;
}

// Called repeatedly when this Command is scheduled to run
void ControlIntakeActuator::Execute() {
  Robot::intakeSubsystem->SetIntakeAngle(targetAngle);
  if(Robot::intakeSubsystem->GetAngleError() < kINTAKE_ANGLE_TOLERANCE) {
    isDone = true;
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ControlIntakeActuator::IsFinished() { 
  return isDone; 
}

// Called once after isFinished returns true
void ControlIntakeActuator::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ControlIntakeActuator::Interrupted() {}
