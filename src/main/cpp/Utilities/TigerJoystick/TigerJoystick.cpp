#include "Utilities/TigerJoystick/TigerJoystick.h"

/**
 * Constructor for a joystick. Just pass in the port that appears in the
 * driver station window
 * @param port port of joystick
 */
TigerJoystick::TigerJoystick(int port) {
	joystick = std::make_unique<frc::Joystick>(port);
	aButton = std::make_unique<frc::JoystickButton>(joystick.get(), 1);
	bButton = std::make_unique<frc::JoystickButton>(joystick.get(), 2);
	xButton = std::make_unique<frc::JoystickButton>(joystick.get(), 3);
	yButton = std::make_unique<frc::JoystickButton>(joystick.get(), 4);
	leftShoulderButton = std::make_unique<frc::JoystickButton>(joystick.get(), 5);
	rightShoulderButton= std::make_unique<frc::JoystickButton>(joystick.get(), 6);
	selectButton = std::make_unique<frc::JoystickButton>(joystick.get(), 7);
	startButton = std::make_unique<frc::JoystickButton>(joystick.get(), 8);
	leftStickButton = std::make_unique<frc::JoystickButton>(joystick.get(), 9);
	rightStickButton = std::make_unique<frc::JoystickButton>(joystick.get(), 10);
	leftTrigger = std::make_unique<TigerLeftTrigger>(joystick.get(), 3);
	rightTrigger = std::make_unique<TigerRightTrigger>(joystick.get(), 3);
}

/**
 * Destructor and clean up
 */
TigerJoystick::~TigerJoystick() {
	joystick.reset();
	aButton.reset();
	bButton.reset();
	xButton.reset();
	yButton.reset();
	leftShoulderButton.reset();
	rightShoulderButton.reset();
	selectButton.reset();
	leftStickButton.reset();
	rightStickButton.reset();
	leftTrigger.reset();
	rightTrigger.reset();
}

double TigerJoystick::GetLeftXAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(0));
}

double TigerJoystick::GetLeftYAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(1));
}

double TigerJoystick::GetRightXAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(4));
}

double TigerJoystick::GetRightYAxis() {
	return DeadBandJoystick(joystick->GetRawAxis(5));
}

double TigerJoystick::GetLeftTriggerValue() {
	return leftTrigger->GetTriggerValue();
}

double TigerJoystick::GetRightTriggerValue() {
	return rightTrigger->GetTriggerValue();
}

bool TigerJoystick::GetLeftTriggerPressed() {
	return leftTrigger->Get();
}

bool TigerJoystick::GetRightTriggerPressed() {
	return rightTrigger->Get();
}

TigerLeftTrigger* TigerJoystick::GetLeftTrigger() {
	return leftTrigger.get();
}

TigerRightTrigger* TigerJoystick::GetRightTrigger() {
	return rightTrigger.get();
}

/**
 * Makes sure we dont return really small, garbage values like .0001
 * @param axis axis of joystick we want to look at
 * @return the good value of joystick
 */
double TigerJoystick::DeadBandJoystick(double axis) {
	if(axis > -0.20 && axis < 0.20) {
		axis = 0;
	}
	else {
		axis = axis * fabs(axis);
	}
	return axis;
}
