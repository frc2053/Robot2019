/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Intake/ControlIntakeWheels.h"
#include "Robot.h"
#include "RobotMap.h"

ControlIntakeWheels::ControlIntakeWheels(double time, double speed, bool currentCheck) {
  Requires(Robot::intakeSubsystem.get());
  timer = std::make_unique<frc::Timer>();
  timeCurrent = 0;
  isDone = false;
  inputSpeed = speed;
  timeTarget = time;
  timer->Reset();
  timer->Start();
	current = 0;
	isCheckCurrentSpike = currentCheck;
}

// Called just before this Command runs the first time
void ControlIntakeWheels::Initialize() {
  	isDone = false;
  	timeCurrent = 0;
	timer->Reset();
	timer->Start();
	isCheckCurrentSpike = false;
}

// Called repeatedly when this Command is scheduled to run
void ControlIntakeWheels::Execute() {
	Robot::intakeSubsystem->SetIntakeWheelSpeed(inputSpeed);
	isDone = true;
}

// Make this return true when this Command no longer needs to run execute()
bool ControlIntakeWheels::IsFinished() { 
  return isDone; 
}

// Called once after isFinished returns true
void ControlIntakeWheels::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ControlIntakeWheels::Interrupted() {}
