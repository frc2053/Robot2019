/**
 * \file DriveCommand.h
 * \brief The lets us drive around!
 * \author Drew Williams
 */

#pragma once

#include <frc/commands/Command.h>

class DriveCommand : public frc::Command {
public:
	DriveCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	void GetInputs();
	void SetAngleFromInput();
	void RotateCommand();
	void CallToTankDrive();
	void CheckRotateOverride();
private:
	double throttleAxis;
	double turnAxis;
	double currentYaw;
	double setAngle;
	double finalRotVal;
	bool isAPressed;
	bool isBPressed;
	bool isXPressed;
	bool isYPressed;
	bool isRightShoulderPressed;
	bool isRotDone;
};