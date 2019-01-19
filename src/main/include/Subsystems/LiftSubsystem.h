/*----------------------------------------------------------------------------*/
/* Catherine Deskur made this                                                 */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>

class LiftSubsystem : public frc::Subsystem {
 private:
 std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> FootDriverTalon;
 std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> LegDriverTalon;
 std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> LegFollowerTalon;
 int DistanceToTicks(double distance);

 public:
  LiftSubsytem();
  double GetPosition();
  void SetPosition(double position);
  void SetFootSpeed(double speed);
  void SetLegSpeed(double speed);
};
