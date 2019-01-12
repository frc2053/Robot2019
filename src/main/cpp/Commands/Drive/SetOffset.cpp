#include "Commands/Drive/SetOffset.h"
#include "Robot.h"

/**
 * Constructor for our command to change the offset
 * @param input angle in degrees of offset
 */
SetOffset::SetOffset(float input) {
	Requires(Robot::drivebaseSubsystem.get());
	isDone = false;
	inputYaw = input;
}

void SetOffset::Initialize() {
	isDone = false;
}

/**
 * Instantly ends after setting yaw
 */
void SetOffset::Execute() {
	Robot::drivebaseSubsystem->GetTigerDrive()->SetAdjYaw(inputYaw);
	isDone = true;
}

bool SetOffset::IsFinished() {
	return isDone;
}

void SetOffset::End() {

}

void SetOffset::Interrupted() {

}
