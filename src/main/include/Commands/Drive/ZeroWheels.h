/**
 * \file ZeroWheels.h
 * \brief Zeros the swerve modules as a command
 * \author Drew Williams
 */
#pragma once

#include <Commands/Command.h>

/**
 * Representation of a command that zeros the wheels
 */
class ZeroWheels: public frc::Command {
public:
	ZeroWheels();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
};
