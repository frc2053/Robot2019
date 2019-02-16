/*----------------------------------------------------------------------------*/
/* CATHERINE WROTE THIS CODE                                                  */
/*----------------------------------------------------------------------------*/

#include "Subsystems/LiftSubsystem.h"
#include "Robot.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

LiftSubsystem::LiftSubsystem() : Subsystem("LiftSubsystem") {
  FootDriverTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kFOOT_TALON_ID);
  LegDriverTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kLEG_LEADER_TALON_ID);
  LegFollowerTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(Robot::robotMap->kLEG_FOLLOWER_TALON_ID);

  LegFollowerTalon->Follow(*LegDriverTalon.get());
  LegDriverTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
}

int LiftSubsystem::DistanceToTicks(double distance){
  int ticks = distance * Robot::robotMap->kELEVATORTICKS_PER_INCH;
  return ticks;
}

double LiftSubsystem::GetPosition() {
  double position = LegDriverTalon->GetSelectedSensorPosition(0);
  return position;
}

void LiftSubsystem::SetPosition(double position){
  int ticks = DistanceToTicks(position);
  LegDriverTalon->Set(ctre::phoenix::motorcontrol::ControlMode::Position, ticks);
}

void LiftSubsystem::SetFootSpeed(double speed){
  FootDriverTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void LiftSubsystem::SetLegSpeed(double speed){
  LegDriverTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

int LiftSubsystem::GetLegClosedLoopError() {
  return LegDriverTalon->GetClosedLoopError(0);
}
