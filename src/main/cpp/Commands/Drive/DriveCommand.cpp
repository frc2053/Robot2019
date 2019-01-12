#include "Commands/Drive/DriveCommand.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotMap.h"
#include <iostream>

/**
 * Constructor for DriveCommand
 * We set up our button states here
 * And make sure we Require() our subsystem so we can take authority
 * over other commands that may run on the drivebase
 */
DriveCommand::DriveCommand() {
	Requires(Robot::drivebaseSubsystem.get());

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
	isRotDone = true;
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
	isRotDone = true;
}

/**
 * Executes once every .2 seconds. We do all our calculations here
 * to tell the robot how to drive
 */
void DriveCommand::Execute() {
	GetInputs();
	currentYaw = Robot::drivebaseSubsystem->GetTigerDrive()->GetAdjYaw();
	isRotDone = Robot::drivebaseSubsystem->GetTigerDrive()->GetIsRotDone();

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
		Robot::drivebaseSubsystem->GetTigerDrive()->rotateController->SetSetpoint(180);
	}
	if(isBPressed) {
		Robot::drivebaseSubsystem->GetTigerDrive()->rotateController->SetSetpoint(90);
		setAngle = 90;
	}
	if(isXPressed) {
		Robot::drivebaseSubsystem->GetTigerDrive()->rotateController->SetSetpoint(-90);
		setAngle = -90;
	}
	if(isYPressed) {
		Robot::drivebaseSubsystem->GetTigerDrive()->rotateController->SetSetpoint(0);
		setAngle = 0;
	}
}

/**
 * make sure we actually want to rotate and we dont get into weird
 * states where we double rotate
 */
void DriveCommand::RotateCommand()
{
	if(((isYPressed == true || isXPressed == true || isAPressed == true || isBPressed == true) && isRotDone == true) || (isRotDone == false))
	{
		finalRotVal = Robot::drivebaseSubsystem->GetTigerDrive()->CalculateRotationValue(setAngle, 1);
	}
}

/**
 * make sure we are checking if we want to cancel rotating
 */
void DriveCommand::CheckRotateOverride() {
	if(Robot::drivebaseSubsystem->GetTigerDrive()->GetIsRotDoneOverride())
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
		Robot::drivebaseSubsystem->GetTigerDrive()->SetIsRotDoneOverride(false);
		Robot::drivebaseSubsystem->MecDrive(xAxis, -yAxis, finalRotVal, currentYaw);
	}
	else
	{
		Robot::drivebaseSubsystem->GetTigerDrive()->SetIsRotDoneOverride(true);
		Robot::drivebaseSubsystem->GetTigerDrive()->SetIsRotDone(true);
		Robot::drivebaseSubsystem->MecDrive(xAxis, -yAxis, rotAxis, currentYaw);
	}
}
