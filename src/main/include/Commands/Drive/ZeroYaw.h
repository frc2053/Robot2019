/**
 * \file ZeroYaw.h
 * \brief Command to zero the IMU on the robot
 */
#pragma once

#include <Commands/Command.h>

/**
 * A command that zeros the yaw of the robot
 */
class ZeroYaw: public frc::Command
{
public:
	ZeroYaw();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
};
