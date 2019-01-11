/**
 * \file OI.h
 * \brief Holds joystick commands
 * \author Cathy Deskur & Drew Williams
 */

#pragma once

#include "Utilities/TigerJoystick/TigerJoystick.h"

/**
 * This class is initialized in Robot::RobotInit where it then
 * sits and listens for joystick presses and schedules commands
 * if we press a button
 */
class OI {
public:
	OI();
	const std::unique_ptr<TigerJoystick>& GetDriverController();
private:
	std::unique_ptr<TigerJoystick> driverController;
	std::unique_ptr<TigerJoystick> operatorController;
};
