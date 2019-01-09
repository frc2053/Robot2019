#pragma once

#include <frc/commands/Subsystem.h>
#include <unordered_map>
#include "ctre/Phoenix.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/SpeedControllerGroup.h"
#include "AHRS.h"
#include "frc/PIDController.h"
#include "Utilities/TigerDrive/TigerDrive.h"

class DrivebaseSubsystem : public frc::Subsystem {
  public:
    DrivebaseSubsystem();
    void InitDefaultCommand() override;
    void TankDrive(double throttleJoystick, double turnJoystick, bool quickTurnEnabled);
    const std::unique_ptr<TigerDrive>& GetTigerDrive();
    void Periodic() override;
  private:
    void ConfigureTalons();
    std::shared_ptr<WPI_TalonSRX> leftLeaderTalon;
    std::shared_ptr<WPI_TalonSRX> leftFollowerTalon;
    std::shared_ptr<WPI_TalonSRX> rightLeaderTalon;
    std::shared_ptr<WPI_TalonSRX> rightFollowerTalon;
    std::shared_ptr<AHRS> imu;
    std::unique_ptr<TigerDrive> tigerDrive;
    std::unique_ptr<frc::DifferentialDrive> driveSystem;
};