/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Intake/ControlSlapper.h"
#include "Robot.h"
#include "RobotMap.h"

ControlSlapper::ControlSlapper(double intakeAngle) {
  Requires(Robot::intakeSubsystem.get());
  targetAngle = intakeAngle;
  isDone = false;
}

// Called just before this Command runs the first time
void ControlSlapper::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ControlSlapper::Execute() {
  Robot::intakeSubsystem->SetSlapperAngle(targetAngle);
  if(std::abs(Robot::intakeSubsystem->GetSlapperError()) < 5) {
    isDone = true;
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ControlSlapper::IsFinished() { 
  return isDone; 
}

// Called once after isFinished returns true
void ControlSlapper::End() {
// put stuff here
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ControlSlapper::Interrupted() {

}
