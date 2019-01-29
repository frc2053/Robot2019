/**
 * \file OI.cpp
 * \brief Holds joystick commands
 * \author Drew Williams
 */

#include "OI.h"
#include "Robot.h"

#include "Commands/Intake/ControlIntakeWrist.h"
#include "Commands/Intake/ControlIntakeWheels.h"
#include "Commands/Intake/ControlFlapper.h"

/**
 * \brief OI Constructor
 *
 * Here we create our joysticks and set up listeners for button
 * presses. This is all we need to use the controllers.
 */
OI::OI() {
	driverController = std::make_unique<TigerJoystick>(Robot::robotMap->kDRIVER_CONTROLLER_PORT);
	operatorController = std::make_unique<TigerJoystick>(Robot::robotMap->kOPERATOR_CONTROLLER_PORT);

	//INTAKE
	operatorController->yButton->WhenPressed(new ControlIntakeWrist(Robot::robotMap->kINTAKE_ANGLE_UP));
	operatorController->aButton->WhenPressed(new ControlIntakeWrist(Robot::robotMap->kINTAKE_ANGLE_BALL));

	operatorController->xButton->WhenActive(new ControlIntakeWheels(0, Robot::robotMap->kINTAKE_SPEED));
	operatorController->xButton->WhenReleased(new ControlIntakeWheels(0, 0));

	operatorController->bButton->WhenActive(new ControlFlapper(Robot::robotMap->kFLAPPER_DOWN_ANGLE));
	operatorController->bButton->WhenReleased(new ControlFlapper(Robot::robotMap->kFLAPPER_UP_ANGLE));
}

const std::unique_ptr<TigerJoystick>& OI::GetDriverController() {
	return driverController;
}

const std::unique_ptr<TigerJoystick>& OI::GetOperatorController() {
	return operatorController;
}
