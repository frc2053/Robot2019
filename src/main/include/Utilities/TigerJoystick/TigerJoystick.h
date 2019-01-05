/**
 * \file TigerJoystick.h
 * \brief Allows us to call commands from joystick buttons.
 *  We can also just grab the state of buttons
 * \author Drew Williams
 */

#pragma once

#include <frc/Joystick.h>
#include <frc/buttons/JoystickButton.h>
#include "TigerLeftTrigger.h"
#include "TigerRightTrigger.h"

/**
 * A class that represents a joystick like an Xbox Controller
 * We can get various states of buttons and stuff like that.
 * Usually holds two triggers to represent analog triggers
 */
class TigerJoystick {
public:
	TigerJoystick(int port);
	virtual ~TigerJoystick();

	double GetLeftXAxis();
	double GetLeftYAxis();
	double GetRightXAxis();
	double GetRightYAxis();
	double GetLeftTriggerValue();
	double GetRightTriggerValue();
	bool GetLeftTriggerPressed();
	bool GetRightTriggerPressed();
	TigerLeftTrigger* GetLeftTrigger();
	TigerRightTrigger* GetRightTrigger();
	std::unique_ptr<frc::JoystickButton> aButton;
	std::unique_ptr<frc::JoystickButton> bButton;
	std::unique_ptr<frc::JoystickButton> xButton;
	std::unique_ptr<frc::JoystickButton> yButton;
	std::unique_ptr<frc::JoystickButton> leftShoulderButton;
	std::unique_ptr<frc::JoystickButton> rightShoulderButton;
	std::unique_ptr<frc::JoystickButton> startButton;
	std::unique_ptr<frc::JoystickButton> selectButton;
	std::unique_ptr<frc::JoystickButton> leftStickButton;
	std::unique_ptr<frc::JoystickButton> rightStickButton;
private:
	double DeadBandJoystick(double axis);
	std::unique_ptr<TigerLeftTrigger> leftTrigger;
	std::unique_ptr<TigerRightTrigger> rightTrigger;
	std::unique_ptr<frc::Joystick> joystick;
};
