/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Lift/ControlLeg.h"
#include "Robot.h"

ControlLeg::ControlLeg(double height) {
	Requires(Robot::liftSubsystem.get());
	heightTarget = height;
}

void ControlLeg::Initialize() {
	isDone = false;
}

void ControlLeg::Execute() {
	Robot::liftSubsystem->SetPosition(heightTarget);
	if (Robot::liftSubsystem->GetLegClosedLoopError() < 30) {
		isDone = true;
	}
}

bool ControlLeg::IsFinished() {
	return isDone;
}

void ControlLeg::End() {

}

void ControlLeg::Interrupted() {

}
