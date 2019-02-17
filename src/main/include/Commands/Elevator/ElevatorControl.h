#pragma once

#include <frc/commands/Command.h>

class ElevatorControl : public frc::Command {
public:
	ElevatorControl();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
	enum ELEVATOR_POSITION
	{
		GROUND = 0,
		LEVEL_ONE = 1,
		LEVEL_TWO = 2,
		LEVEL_THREE = 3
	};
private:
	ELEVATOR_POSITION currentState;
	double yAxis;
	bool isRightShoulderPressed;
	bool isLeftShoulderPressed;
	bool manualControl;
	bool lastStateRightShoulder;
	bool lastStateLeftShoulder;
};