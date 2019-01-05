#include "Commands/Drive/ZeroWheels.h"
#include "Robot.h"

ZeroWheels::ZeroWheels() {
	Requires(Robot::swerveSubsystem.get());
	isDone = false;
}

void ZeroWheels::Initialize() {
	isDone = false;
}

/**
 * Calibrates the wheels and ends immediately.
 */
void ZeroWheels::Execute() {
	Robot::swerveSubsystem->CalibrateWheels();
	isDone = true;
}

bool ZeroWheels::IsFinished() {
	return isDone;
}

void ZeroWheels::End() {

}

void ZeroWheels::Interrupted() {

}
