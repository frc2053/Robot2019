/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/LiftSubsystem.h"
#include "RobotMap.h"
LiftSubsystem::LiftSubsystem() : Subsystem("LiftSubsystem") {
  FootDriverTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kFOOT_TALON_ID);
  LegDriverTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kLEG_LEADER_TALON_ID);
  LegFollowerTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kLEG_FOLLOWER_TALON_ID);
}

double LiftSubsystem::GetPosition() {

}


