#pragma once

#include <frc/commands/Command.h>
#include <frc/Timer.h>

class ControlLeg : public frc::Command {
public:
	ControlLeg(double height);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	bool isDone;
	double heightTarget;
};
