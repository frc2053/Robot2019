/**
 * \file OI.cpp
 * \brief Holds joystick commands
 * \author Drew Williams
 */

#include "OI.h"
#include "RobotMap.h"

/**
 * \brief OI Constructor
 *
 * Here we create our joysticks and set up listeners for button
 * presses. This is all we need to use the controllers.
 */
OI::OI() {
	driverController = std::make_unique<TigerJoystick>(kDRIVER_CONTROLLER_PORT);
	operatorController = std::make_unique<TigerJoystick>(kOPERATOR_CONTROLLER_PORT);
}

const std::unique_ptr<TigerJoystick>& OI::GetDriverController() {
	return driverController;
}
