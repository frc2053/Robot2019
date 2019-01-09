#include "subsystems/DrivebaseSubsystem.h"
#include "commands/Drive/DriveCommand.h"
#include "RobotMap.h"
#include "frc/smartDashboard/SmartDashboard.h"

DrivebaseSubsystem::DrivebaseSubsystem() : frc::Subsystem("DrivebaseSubsystem") {
  leftLeaderTalon = std::make_shared<WPI_TalonSRX>(DRIVEBASE_LEFT_SIDE_LEADER_ID);
  leftFollowerTalon = std::make_shared<WPI_TalonSRX>(DRIVEBASE_LEFT_SIDE_FOLLOWER_ID);
  rightLeaderTalon = std::make_shared<WPI_TalonSRX>(DRIVEBASE_RIGHT_SIDE_LEADER_ID);
  rightFollowerTalon = std::make_shared<WPI_TalonSRX>(DRIVEBASE_RIGHT_SIDE_FOLLOWER_ID);
  imu = std::make_shared<AHRS>(frc::SPI::Port::kMXP);
  tigerDrive = std::make_unique<TigerDrive>(imu);
  driveSystem = std::make_unique<frc::DifferentialDrive>(*leftLeaderTalon, *rightLeaderTalon);
}

void DrivebaseSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DriveCommand());
}

void DrivebaseSubsystem::TankDrive(double throttleJoystick, double turnJoystick, bool quickTurnEnabled) {
  driveSystem->CurvatureDrive(throttleJoystick, turnJoystick, quickTurnEnabled);
}

void DrivebaseSubsystem::ConfigureTalons() {
  //make sure when you go forward on joystick, you get a positive number
  //and that ALL Talons turn green
  //if not change below
  leftLeaderTalon->ConfigFactoryDefault();
  leftFollowerTalon->ConfigFactoryDefault();
  rightLeaderTalon->ConfigFactoryDefault();
  rightFollowerTalon->ConfigFactoryDefault();

  leftFollowerTalon->Follow(*leftLeaderTalon);
  rightFollowerTalon->Follow(*rightLeaderTalon);

  //only change these
  leftLeaderTalon->SetInverted(false);
  leftFollowerTalon->SetInverted(false);
  rightLeaderTalon->SetInverted(false);
  rightFollowerTalon->SetInverted(false);
  //change these if you have encoders and make sure they increase in value when talons are green
  leftLeaderTalon->SetSensorPhase(true);
  rightLeaderTalon->SetSensorPhase(true);

  //DO NOT CHANGE!!
  driveSystem->SetRightSideInverted(false);
}

void DrivebaseSubsystem::Periodic() {
  SmartDashboard::PutNumber("Left Encoder Value", leftLeaderTalon->GetSelectedSensorPosition());
  SmartDashboard::PutNumber("Right Encoder Value", rightLeaderTalon->GetSelectedSensorPosition());
}

const std::unique_ptr<TigerDrive>& DrivebaseSubsystem::GetTigerDrive() {
  return tigerDrive;
}