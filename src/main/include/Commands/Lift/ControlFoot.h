/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include <frc/Timer.h>

class ControlFoot : public frc::Command {
 public:
  ControlFoot(double time = 0, double speed = 0);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
 private:
  double timeTarget;
	double timeCurrent;
	double inputSpeed;
	bool isDone;
  std::unique_ptr<frc::Timer> timer;
};
