#include "SetOffset.h"
#include "../../Robot.h"

/**
 * Constructor for our command to change the offset
 * @param input angle in degrees of offset
 */
SetOffset::SetOffset(float input) {
	Requires(Robot::swerveSubsystem.get());
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
	Robot::swerveSubsystem->GetTigerDrive()->SetAdjYaw(inputYaw);
	isDone = true;
}

bool SetOffset::IsFinished() {
	return isDone;
}

void SetOffset::End() {

}

void SetOffset::Interrupted() {

}
