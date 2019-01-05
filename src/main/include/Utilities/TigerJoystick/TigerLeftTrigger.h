/**
 * \file TigerLeftTrigger.h
 * \brief A class that represents the left XBox Controller Triggers
 * \author Drew Williams
 */

#pragma once

#include <frc/Joystick.h>
#include <frc/buttons/Trigger.h>

/**
 * Inherits from frc::Trigger so we can activate commands from the trigger
 */
class TigerLeftTrigger : public frc::Trigger {
public:
	TigerLeftTrigger(frc::Joystick* joy, int axis);
	bool Get();
	double GetTriggerValue();
private:
	double Deadband(double axis);

	int joystickAxis;
	float joystickValue;
	frc::Joystick* joystick;
};
