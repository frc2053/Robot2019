/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "Utilities/TigerMecanum/DriveController.h"
#include "Utilities/Math/PIDController2481.h"
#include "Utilities/Math/InterpolatingMap.h"
#include "Utilities/TigerMecanum/PathLoader.h"

class FollowPath : public frc::Command {
public:
  FollowPath(std::string path);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
private:
  DriveController* m_driveController;
  bool m_skip;
  std::string m_filePath;
protected:
  Path2D& m_path;
};
