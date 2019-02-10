/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include <string>
#include <vector>

class FollowPathVelocity : public frc::Command {
 public:
  FollowPathVelocity(std::string pathName);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
private:
  bool isDone;
  std::vector<double> flVelocityList;
  std::vector<double> frVelocityList;
  std::vector<double> blVelocityList;
  std::vector<double> brVelocityList;
  int tick;
};
