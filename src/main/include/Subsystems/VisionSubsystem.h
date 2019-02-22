/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>


class VisionSubsystem : public frc::Subsystem {
 private:
  int translationX;
  int translationY;
  std::shared_ptr<NetworkTable> table;
 public:
  VisionSubsystem();
  void InitDefaultCommand() override;
  virtual void Periodic();
  int GetTranslationX();
  int GetTranslationY();
};
