/**
 * \file OI.cpp
 * \brief Holds joystick commands
 * \author Drew Williams
 */

#include "OI.h"
#include "Robot.h"

#include "Commands/Intake/ControlIntakeWrist.h"
#include "Commands/Intake/ControlIntakeWheels.h"
#include "Commands/Intake/ControlSlapper.h"
#include "Commands/Lift/ControlLeg.h"
#include "Commands/Lift/ControlFoot.h"

#include <frc/smartdashboard/SmartDashboard.h>

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
	operatorController->selectButton->WhenPressed(new ControlIntakeWrist(Robot::robotMap->kINTAKE_ANGLE_UP));
	operatorController->startButton->WhenPressed(new ControlIntakeWrist(Robot::robotMap->kINTAKE_ANGLE_BALL));
	
	operatorController->aButton->WhenActive(new ControlIntakeWheels(0, Robot::robotMap->kINTAKE_SPEED, false));
	operatorController->aButton->WhenInactive(new ControlIntakeWheels(0, 0, false));

	operatorController->bButton->WhenActive(new ControlIntakeWheels(0, -Robot::robotMap->kINTAKE_SPEED, false));
	operatorController->bButton->WhenInactive(new ControlIntakeWheels(0, 0, false));

	//operatorController->xButton->WhenPressed(new ControlLeg(25067));
	//operatorController->yButton->WhenPressed(new ControlLeg(0));

	operatorController->rightShoulderButton->WhenPressed(new ControlSlapper(Robot::robotMap->kSLAPPER_RELEASE_TICKS));
	operatorController->leftShoulderButton->WhenPressed(new ControlSlapper(Robot::robotMap->kSLAPPER_DOWN_TICKS));
	operatorController->leftShoulderButton->WhenReleased(new ControlSlapper(Robot::robotMap->kSLAPPER_UP_TICKS));

	operatorController->xButton->WhenPressed(new ControlFoot(0, -1));
	operatorController->xButton->WhenReleased(new ControlFoot(0, 0));

}

const std::unique_ptr<TigerJoystick>& OI::GetDriverController() {
	return driverController;
}

const std::unique_ptr<TigerJoystick>& OI::GetOperatorController() {
	return operatorController;
}
