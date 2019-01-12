#include "Commands/Drive/ZeroYaw.h"
#include "Robot.h"

ZeroYaw::ZeroYaw()
{
	Requires(Robot::drivebaseSubsystem.get());
	isDone = false;
}

void ZeroYaw::Initialize()
{
	isDone = false;
}

/**
 * Zeros the IMU and ends
 */
void ZeroYaw::Execute()
{
	isDone = false;
	Robot::drivebaseSubsystem->GetTigerDrive()->imu->ZeroYaw();
	isDone = true;
}

bool ZeroYaw::IsFinished()
{
	return isDone;
}

void ZeroYaw::End()
{

}

void ZeroYaw::Interrupted()
{

}
