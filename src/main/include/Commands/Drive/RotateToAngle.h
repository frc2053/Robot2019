/**
 * \file RotateToAngle.h
 * \brief Allows the robot to rotate to a specififed angle in auto mode.
 * \author Drew Williams
 * \
 */

#pragma once

#include <frc/commands/Command.h>

class RotateToAngle : frc::Command {
public:
	RotateToAngle(double angleToRateTo);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
	double inputAngle;
	double currentYaw;
};
