/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Intake/ControlIntakeWheels.h"
#include "Robot.h"

ControlIntakeWheels::ControlIntakeWheels(double time, double speed) {
  Requires(Robot::intakeSubsystem.get());
  timer = std::make_unique<frc::Timer>();
  timeCurrent = 0;
  isDone = false;
  inputSpeed = speed;
  timeTarget = time;
  timer->Reset();
  timer->Start();
}

// Called just before this Command runs the first time
void ControlIntakeWheels::Initialize() {
  isDone = false;
	timeCurrent = 0;
	timer->Reset();
	timer->Start();
}

// Called repeatedly when this Command is scheduled to run
void ControlIntakeWheels::Execute() {
  if(inputSpeed == 0) {
		Robot::intakeSubsystem->SetIntakeWheelSpeed(inputSpeed);
		isDone = true;
	}
	else {
		timeCurrent = timer->Get();
		if(timeTarget == 0) {
			Robot::intakeSubsystem->SetIntakeWheelSpeed(inputSpeed);
			isDone = false;
		}
		else {
			if(timeCurrent >= timeTarget) {
				Robot::intakeSubsystem->SetIntakeWheelSpeed(0);
				isDone = true;
			}
			else {
				Robot::intakeSubsystem->SetIntakeWheelSpeed(inputSpeed);
				isDone = false;
			}
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool ControlIntakeWheels::IsFinished() { 
  return isDone; 
}

// Called once after isFinished returns true
void ControlIntakeWheels::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ControlIntakeWheels::Interrupted() {}
