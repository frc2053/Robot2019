/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Elevator/ElevatorControl.h"
#include "Robot.h"
#include "RobotMap.h"

ElevatorControl::ElevatorControl() {
	Requires(Robot::elevatorSubsystem.get());
	yAxis = 0;
	isRightShoulderPressed = false;
	isLeftShoulderPressed = false;
	manualControl = false;
	currentState = ELEVATOR_POSITION::GROUND;
}

// Called just before this Command runs the first time
void ElevatorControl::Initialize() {
	yAxis = 0;
	isRightShoulderPressed = false;
	isLeftShoulderPressed = false;
	currentState = ELEVATOR_POSITION::GROUND;
	manualControl = false;
}

// Called repeatedly when this Command is scheduled to run
void ElevatorControl::Execute() {
	yAxis = Robot::oi->GetOperatorController()->GetLeftYAxis();
	isRightShoulderPressed = Robot::oi->GetOperatorController()->rightShoulderButton->Get();
	isLeftShoulderPressed = Robot::oi->GetOperatorController()->leftShoulderButton->Get();

	if (yAxis != 0) {
		manualControl = true;
	}
	else {
		manualControl = false;
	}

	if (isRightShoulderPressed) {
		currentState = (ELEVATOR_POSITION)((int)currentState++);
	}
	if (isLeftShoulderPressed) {
		currentState = (ELEVATOR_POSITION)((int)currentState--);
	}

	if (manualControl) {
		Robot::elevatorSubsystem->RunElevatorMotor(yAxis);
	}
	else {
		switch(currentState)
		{
		case GROUND:
			Robot::elevatorSubsystem->GoToHeight(kELEVATOR_GROUND);
			break;
		case LEVEL_ONE:
			Robot::elevatorSubsystem->GoToHeight(kELEVATOR_LEVEL_ONE);
			break;
		case LEVEL_TWO:
			Robot::elevatorSubsystem->GoToHeight(kELEVATOR_LEVEL_TWO);
			break;
		case LEVEL_THREE:
			Robot::elevatorSubsystem->GoToHeight(kELEVATOR_LEVEL_THREE);
			break;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorControl::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ElevatorControl::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorControl::Interrupted() {

}
