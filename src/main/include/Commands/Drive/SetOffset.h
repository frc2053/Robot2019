/**
 * \file SetOffset.h
 * \brief Allows us to change imu orientation
 * \author Drew Williams
 */
#pragma once

#include <Commands/Command.h>

/**
 * A command that ends instantly that
 * changes the yaw offset of the robot.
 */
class SetOffset : public frc::Command {
public:
	SetOffset(float input);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
	double inputYaw;
};
