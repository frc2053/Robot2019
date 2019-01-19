/*----------------------------------------------------------------------------*/
/* CATHERINE WROTE THIS CODE                                                  */
/*----------------------------------------------------------------------------*/

#include "Subsystems/LiftSubsystem.h"
#include "RobotMap.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

LiftSubsystem::LiftSubsystem() : Subsystem("LiftSubsystem") {
  FootDriverSpark = std::make_unique<rev::CANSparkMax>(kFOOT_SPARK_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
  LegDriverTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kLEG_LEADER_TALON_ID);
  LegFollowerTalon = std::make_unique<ctre::phoenix::motorcontrol::can::TalonSRX>(kLEG_FOLLOWER_TALON_ID);

  LegFollowerTalon->Follow(*LegDriverTalon.get());
  LegDriverTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
}

int LiftSubsystem::DistanceToTicks(double distance){
  int ticks = distance * kELEVATORTICKS_PER_INCH;
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
  FootDriverSpark->Set(speed);
}

void LiftSubsystem::SetLegSpeed(double speed){
  LegDriverTalon->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

