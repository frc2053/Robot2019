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

  FootDriverTalon->ConfigFactoryDefault();
  LegDriverTalon->ConfigFactoryDefault();

  LegDriverTalon->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::CTRE_MagEncoder_Relative);
  LegDriverTalon->SetSelectedSensorPosition(0);
  LegDriverTalon->SetSensorPhase(true);

  LegDriverTalon->Config_kF(0, Robot::robotMap->kLEG_F);
  LegDriverTalon->Config_kP(0, Robot::robotMap->kLEG_P);
  LegDriverTalon->Config_kI(0, Robot::robotMap->kLEG_I);
  LegDriverTalon->Config_kD(0, Robot::robotMap->kLEG_D);
}

double LiftSubsystem::GetPosition() {
  double position = LegDriverTalon->GetSelectedSensorPosition(0);
  return position;
}

void LiftSubsystem::SetPosition(double position){
  std::cout << "curr pos: " << LegDriverTalon->GetSelectedSensorPosition() << "\n";
  std::cout << "pos set: " << position << "\n";
  LegDriverTalon->Set(ctre::phoenix::motorcontrol::ControlMode::Position, position);
  std::cout << "talon setpt: " << LegDriverTalon->GetClosedLoopTarget() << "\n";
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

void LiftSubsystem::Periodic() {
  double legControlAxis = Robot::oi->GetOperatorController()->GetRightYAxis();
  SetLegSpeed(legControlAxis);
}
