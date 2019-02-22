/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Drive/DriveToVisionTarget.h"
#include "Robot.h"

DriveToVisionTarget::DriveToVisionTarget() {
  Requires(Robot::drivebaseSubsystem.get());
  isDone = false;
}

void DriveToVisionTarget::Initialize() {
  isDone = false;
}

void DriveToVisionTarget::Execute() {
  double strafeCmd = Robot::visionSubsystem->GetTranslationX() * Robot::robotMap->kVISION_STEER_P;
  double fowCmd = (Robot::robotMap->kVISION_TARGET_AREA - Robot::visionSubsystem->GetTargetArea()) * Robot::robotMap->kVISION_FOW_P;
  if(fowCmd > Robot::robotMap->kVISION_MAX_SPEED) {
    fowCmd = Robot::robotMap->kVISION_MAX_SPEED;
  }
  Robot::drivebaseSubsystem->MecDrive(strafeCmd, fowCmd, 0, 0);

  if(Robot::visionSubsystem->GetTargetArea() > Robot::robotMap->kVISION_TARGET_AREA) {
    isDone = true;
  }
  else {
    isDone = false;
  }
}

bool DriveToVisionTarget::IsFinished() { 
  return isDone;
}

void DriveToVisionTarget::End() {

}

void DriveToVisionTarget::Interrupted() {

}
