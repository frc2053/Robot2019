/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Elevator/ElevatorControl.h"
#include "Robot.h"
#include <iostream>
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>

ElevatorControl::ElevatorControl() {
	Requires(Robot::elevatorSubsystem.get());
	yAxis = 0;
	isRightTriggerPressed = false;
	isLeftTriggerPressed = false;
	manualControl = false;
	lastStateLeftTrigger = false;
	lastStateRightTrigger = false;
	currentState = ELEVATOR_POSITION::GROUND;
}

// Called just before this Command runs the first time
void ElevatorControl::Initialize() {
	yAxis = 0;
	isRightTriggerPressed = false;
	isLeftTriggerPressed = false;
	lastStateLeftTrigger = false;
	lastStateRightTrigger = false;
	currentState = ELEVATOR_POSITION::GROUND;
	manualControl = false;
}

// Called repeatedly when this Command is scheduled to run
void ElevatorControl::Execute() {
	yAxis = Robot::oi->GetOperatorController()->GetLeftYAxis();
	isRightTriggerPressed = Robot::oi->GetOperatorController()->GetRightTriggerPressed();
	isLeftTriggerPressed = Robot::oi->GetOperatorController()->GetLeftTriggerPressed();

	if (yAxis != 0) {
		manualControl = true;
		currentState = HOLD;
	}
	else {
		manualControl = false;
		currentState = ELEVATOR_POSITION::HOLD;
		Robot::elevatorSubsystem->RunElevatorMotor(0);
	}
	
	if (isRightTriggerPressed && (lastStateRightTrigger != true)) {
		if (!(currentState == ELEVATOR_POSITION::LEVEL_THREE)||!(currentState == ELEVATOR_POSITION::HOLD)) {
			currentState = (ELEVATOR_POSITION)(currentState + 1);
			std::cout << "UP!" << std::endl;
		}
		else if(currentState == ELEVATOR_POSITION::HOLD){
			int distance = Robot::elevatorSubsystem->GetElevatorHeight();
			int n = 0;
			if(distance >= Robot::robotMap->kELEVATOR_LEVEL_THREE_HATCH){
				n = 3;
			}
			else if(distance >= Robot::robotMap->kELEVATOR_LEVEL_TWO_HATCH){
				n = 2;
			}
			else if(distance >= Robot::robotMap->kELEVATOR_LEVEL_ONE_HATCH){
				n = 1;
			}
			currentState = (ELEVATOR_POSITION)(n);
			//;)
		}
	}
	if (isLeftTriggerPressed && (lastStateLeftTrigger != true)) {
		if (!(currentState == ELEVATOR_POSITION::GROUND)||!(currentState == ELEVATOR_POSITION::HOLD)) {
			currentState = (ELEVATOR_POSITION)(currentState - 1);
			std::cout << "DOWN!" << std::endl;
		}
		else if(currentState == ELEVATOR_POSITION::HOLD){
			int distance = Robot::elevatorSubsystem->GetElevatorHeight();
			int n = 3;
			if(distance <= Robot::robotMap->kELEVATOR_GROUND){
				n = 0;
			}
			else if(distance <= Robot::robotMap->kELEVATOR_LEVEL_ONE_HATCH){
				n = 1;
			}
			else if(distance <= Robot::robotMap->kELEVATOR_LEVEL_TWO_HATCH){
				n = 2;
			}
			currentState = (ELEVATOR_POSITION)(n);
		}
	}

	if (manualControl) {
		Robot::elevatorSubsystem->RunElevatorMotor(-yAxis * 1);
	}
	
	else {
		switch(currentState)
		{
		case GROUND:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_GROUND);
			break;
		case LEVEL_ONE:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_LEVEL_ONE_HATCH);
			break;
		case LEVEL_TWO:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_LEVEL_TWO_HATCH);
			break;
		case LEVEL_THREE:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_LEVEL_THREE_HATCH);
			break;
		case HOLD:
			Robot::elevatorSubsystem->RunElevatorMotor(0);
			break;			
		}
	}
	
	

	lastStateLeftTrigger = isLeftTriggerPressed;
	lastStateRightTrigger = isRightTriggerPressed;
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
