/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Lift/ControlFoot.h"
#include "Robot.h"

ControlFoot::ControlFoot(double time, double speed) {
  Requires(Robot::liftSubsystem.get());
  timer = std::make_unique<frc::Timer>();
  timeCurrent = 0;
  isDone = false;
  inputSpeed = speed;
  timeTarget = time;
  timer->Reset();
  timer->Start();
}

void ControlFoot::Initialize() {
  isDone = false;
	timeCurrent = 0;
	timer->Reset();
	timer->Start();
}

void ControlFoot::Execute() {
  if(inputSpeed == 0) {
		Robot::liftSubsystem->SetPosition(inputSpeed);
		isDone = true;
	}
	else {
		timeCurrent = timer->Get();
		if(timeTarget == 0) {
			Robot::liftSubsystem->SetPosition(inputSpeed);
			isDone = false;
		}
		else {
			if(timeCurrent >= timeTarget) {
				Robot::liftSubsystem->SetPosition(0);
				isDone = true;
			}
			else {
				Robot::liftSubsystem->SetFootSpeed(inputSpeed);
				isDone = false;
			}
		}
	}
}

bool ControlFoot::IsFinished() { 
  return isDone; 
}

void ControlFoot::End() {

}

void ControlFoot::Interrupted() {

}
