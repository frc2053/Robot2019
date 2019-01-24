/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Drive/FollowPath.h"
#include "Robot.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "RobotMap.h"

FollowPath::FollowPath(std::string filePath) : m_path(Robot::pathManager->GetPath(filePath)) {
  Requires(Robot::drivebaseSubsystem.get());
  m_driveController = Robot::drivebaseSubsystem->GetDriveController();
  m_skip = false;
  m_filePath = filePath;
}

// Called just before this Command runs the first time
void FollowPath::Initialize() {
  m_skip = false;
  m_driveController->EnableController();
}

// Called repeatedly when this Command is scheduled to run
void FollowPath::Execute() {
  RigidTransform2D targetPos = m_path.getInterpolated(InterpolatingDouble(TimeSinceInitialized()));
  m_driveController->SetFieldTarget(targetPos);

  SmartDashboard::PutNumber("Path X", targetPos.getTranslation().getX());
  SmartDashboard::PutNumber("Path Y", targetPos.getTranslation().getY());
  SmartDashboard::PutNumber("Path Yaw", targetPos.getRotation().getDegrees());

  RigidTransform2D driveSignal = m_driveController->GetDriveControlSignal();
  Robot::drivebaseSubsystem->MecDrive(driveSignal.getTranslation.getX(), driveSignal.getTranslation().getY(), driveSignal.getRotation().getDegrees(), 0);
}

// Make this return true when this Command no longer needs to run execute()
bool FollowPath::IsFinished() {
  RigidTransform2D lastPoint = m_path.rbegin()->second;
  RigidTransform2D robotPose = Robot::observer->GetLastRobotPose();

  Translation2D errorTranslation = lastPoint.getTranslation().translateBy(robotPose.getTranslation().inverse());
  Rotation2D errorRotation = lastPoint.getRotation().rotateBy(robotPose.getRotation().inverse());

  return ((fabs(errorTranslation.norm()) < kTOLERANCE_POS) && (fabs(errorRotation.getDegrees()) < kTOLERANCE_HEADING)) ||
    (m_path.rbegin()->first.m_value + 2 < TimeSinceInitialized()) ||
    m_skip;
}

// Called once after isFinished returns true
void FollowPath::End() {
  Robot::drivebaseSubsystem->MecDrive(0, 0, 0, 0);
  SmartDashboard::PutNumber("Drive path duration", TimeSinceInitialized());
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FollowPath::Interrupted() {
  End();
}
