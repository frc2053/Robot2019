#include "Commands/Drive/DriveCommand.h"
#include "Robot.h"
#include "RobotMap.h"

/**
 * Constructor for DriveCommand
 * We set up our button states here
 * And make sure we Require() our subsystem so we can take authority
 * over other commands that may run on the drivebase
 */
DriveCommand::DriveCommand() {
	Requires(Robot::swerveSubsystem.get());

	xAxis = 0;
	yAxis = 0;
	rotAxis = 0;
	currentYaw = 0;
	finalRotVal = 0;
	setAngle = 0;
	isLeftStickPressed = false;
	isAPressed = false;
	isBPressed = false;
	isXPressed = false;
	isYPressed = false;
	isRotDone = false;
}

/**
 * Runs when we first start a command
 * We reset our states here just to be safe
 */
void DriveCommand::Initialize() {
	xAxis = 0;
	yAxis = 0;
	rotAxis = 0;
	isAPressed = false;
	isBPressed = false;
	isXPressed = false;
	isYPressed = false;
}

/**
 * Executes once every .2 seconds. We do all our calculations here
 * to tell the robot how to drive
 */
void DriveCommand::Execute() {
	GetInputs();
	currentYaw = Robot::swerveSubsystem->GetTigerDrive()->GetAdjYaw();
	isRotDone = Robot::swerveSubsystem->GetTigerDrive()->GetIsRotDone();

	SetAngleFromInput();
	RotateCommand();
	CheckRotateOverride();
	CallToSwerveDrive();
}

/**
 * We are never done with driving!
 * @return false
 */
bool DriveCommand::IsFinished() {
	return false;
}

/**
 * We dont need to clean up
 */
void DriveCommand::End() {

}

/**
 * Not useful
 */
void DriveCommand::Interrupted() {

}

/**
 * Grabs joystick state
 */
void DriveCommand::GetInputs() {
	xAxis = Robot::oi->GetDriverController()->GetLeftXAxis();
	yAxis = Robot::oi->GetDriverController()->GetLeftYAxis();
	rotAxis = Robot::oi->GetDriverController()->GetRightXAxis();

	isAPressed = Robot::oi->GetDriverController()->aButton->Get();
	isBPressed = Robot::oi->GetDriverController()->bButton->Get();
	isXPressed = Robot::oi->GetDriverController()->xButton->Get();
	isYPressed = Robot::oi->GetDriverController()->yButton->Get();
}

/**
 * Based on the input, give us a target angle and set the PID target
 */
void DriveCommand::SetAngleFromInput() {
	if(isAPressed) {
		setAngle = 180;
	}
	if(isBPressed) {
		setAngle = 90;
	}
	if(isXPressed) {
		setAngle = -90;
	}
	if(isYPressed) {
		setAngle = 0;
	}
	Robot::swerveSubsystem->GetTigerDrive()->SetAngleTarget(setAngle);
}

/**
 * make sure we actually want to rotate and we dont get into weird
 * states where we double rotate
 */
void DriveCommand::RotateCommand()
{
	if(((isYPressed == true|| isXPressed == true || isAPressed == true || isBPressed == true) && isRotDone == true) || (isRotDone == false))
	{
		finalRotVal = Robot::swerveSubsystem->GetTigerDrive()->CalculateRotationValue(setAngle, .5);//kROTATION_RATE_MULTIPLIER);
	}
}

/**
 * make sure we are checking if we want to cancel rotating
 */
void DriveCommand::CheckRotateOverride() {
	if(Robot::swerveSubsystem->GetTigerDrive()->GetIsRotDoneOverride())
	{
		finalRotVal = 0;
	}
}

/**
 * This calls all the swerve drive stuff so we actually move
 */
void DriveCommand::CallToSwerveDrive() {
	if(rotAxis == 0)
	{
		Robot::swerveSubsystem->GetTigerDrive()->SetIsRotDoneOverride(false);
		Robot::swerveSubsystem->SwerveDrive(xAxis, -yAxis, -finalRotVal, currentYaw);
	}
	else
	{
		Robot::swerveSubsystem->GetTigerDrive()->SetIsRotDoneOverride(true);
		Robot::swerveSubsystem->GetTigerDrive()->SetIsRotDone(true);
		Robot::swerveSubsystem->SwerveDrive(xAxis, -yAxis, -rotAxis, currentYaw);
	}
}
