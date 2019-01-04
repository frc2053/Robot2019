#include "RotateToAngle.h"
#include "../../Robot.h"
#include "../../RobotMap.h"

RotateToAngle::RotateToAngle(double angle) {
	inputAngle = angle;
	isDone = false;
	currentYaw = 0;
}

void RotateToAngle::Initialize() {
	isDone = false;
	currentYaw = 0;
}

void RotateToAngle::Execute() {
	currentYaw = Robot::swerveSubsystem->GetTigerDrive()->GetAdjYaw();
	double outputRate = 0;

	Robot::swerveSubsystem->GetTigerDrive()->SetIsRotDoneOverride(false);
	isDone = Robot::swerveSubsystem->GetTigerDrive()->GetIsRotDone();
	Robot::swerveSubsystem->GetTigerDrive()->SetAngleTarget(inputAngle);

	outputRate = Robot::swerveSubsystem->GetTigerDrive()->CalculateRotationValue(inputAngle, kROTATION_RATE_MULTIPLIER);
	Robot::swerveSubsystem->SwerveDrive(0, 0, -outputRate, currentYaw);
}

bool RotateToAngle::IsFinished() {
	return isDone;
}

void RotateToAngle::End() {
	Robot::swerveSubsystem->SwerveDrive(0, 0, 0, currentYaw);
}

void RotateToAngle::Interrupted() {

}
