/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Intake/IntakeBall.h"
#include "Commands/Intake/ControlIntakeActuator.h"
#include "Commands/Intake/ControlIntakeWheels.h"
#include "Robot.h"

IntakeBall::IntakeBall(double intakeAngle, double intakeSpeed) {
  Requires(Robot::intakeSubsystem.get());
  AddParallel(new ControlIntakeActuator(intakeAngle));
  AddParallel(new ControlIntakeWheels(0, intakeSpeed));
}
