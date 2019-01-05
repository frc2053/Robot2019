/**
 * \file TigerRightTrigger.h
 * \brief A class that represents the right XBox Controller Triggers
 * \author Drew Williams
 */

#pragma once

#include <frc/Joystick.h>
#include <frc/buttons/Trigger.h>

/**
 * Inherits from frc::Trigger so we can activate commands from the trigger
 */
class TigerRightTrigger : public frc::Trigger {
public:
	TigerRightTrigger(frc::Joystick* joy, int axis);
	bool Get();
	float GetTriggerValue();
private:
	double Deadband(double axis);

	int joystickAxis;
	float joystickValue;
	frc::Joystick* joystick;
};
