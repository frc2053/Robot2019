#pragma once

#include "Utilities\TigerSwerve\DriveController.h"
#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

class DriveToPosition : public frc::Command {
private:
	int m_onTargetCounter;
	double m_xPos;
	double m_yPos;
	double m_yaw;

public:
	DriveToPosition() {
		Requires(Robot::swerveSubsystem.get());
		SmartDashboard::PutNumber("target x position", 0);
		SmartDashboard::PutNumber("target y position", 0);
		SmartDashboard::PutNumber("target heading", 0);
		m_onTargetCounter = 0;
	}

	~DriveToPosition() {
	}

	void Initialize() {
		m_xPos = SmartDashboard::GetNumber("target x position", 0);
		m_yPos = SmartDashboard::GetNumber("target y position", 0);
		m_yaw = SmartDashboard::GetNumber("target heading", 0);

		Robot::swerveSubsystem->GetDriveController()->SetFieldTarget(RigidTransform2D(Translation2D(m_xPos, m_yPos), Rotation2D::fromDegrees(m_yaw)));
		Robot::swerveSubsystem->GetDriveController()->EnableController();
	}

	void Execute() {
		std::cout << "Executing DriveToPosition!\n";
		RigidTransform2D driveSignal = Robot::swerveSubsystem->GetDriveController()->GetDriveControlSignal();
		Robot::swerveSubsystem->SwerveDrive(driveSignal.getTranslation().getX(),
							driveSignal.getTranslation().getY(),
							driveSignal.getRotation().getDegrees(), Robot::swerveSubsystem->GetTigerDrive()->GetAdjYaw());
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {

		RigidTransform2D lastPoint = RigidTransform2D(Translation2D(m_xPos, m_yPos), Rotation2D::fromDegrees(m_yaw));
		RigidTransform2D robotPose = Robot::swerveSubsystem->GetObserver()->GetLastRobotPose();

		Translation2D errorTranslation = lastPoint.getTranslation().translateBy(robotPose.getTranslation().inverse());
		Rotation2D errorRotation = lastPoint.getRotation().rotateBy(robotPose.getRotation().inverse());

		return (fabs(errorTranslation.norm()) < 3) && (fabs(errorRotation.getDegrees()) < 3);
	}

	void End() {
		Robot::swerveSubsystem->SwerveDrive(0, 0, 0, 0);
	}
};