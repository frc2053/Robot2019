/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem() : Subsystem("VisionSubsystem") {
  translationX = 0;
  translationY = 0;
  targetArea = 0;
  table = nt::NetworkTableInstance::GetDefault().GetTable("vision");
}

void VisionSubsystem::Periodic() {
  table = nt::NetworkTableInstance::GetDefault().GetTable("vision");
  translationX = table->GetNumber("X", 0.0);
  translationY = table->GetNumber("Y", 0.0);
  targetArea = table->GetNumber("Area", 0.0);
}

void VisionSubsystem::InitDefaultCommand() {

}

int VisionSubsystem::GetTranslationX() {
  return translationX;
}

int VisionSubsystem::GetTranslationY() {
  return translationY;
}

double VisionSubsystem::GetTargetArea() {
  return targetArea;
}