#include "Subsystems/DrivebaseSubsystem.h"
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/commands/Command.h>
#include "Commands/Drive/DriveCommand.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

#include <iostream>

/**
 * Constructor for the subsystem. We setup all the talons in here as well as the tiger drive and swerve stuff.
 */
DrivebaseSubsystem::DrivebaseSubsystem() : Subsystem("DrivebaseSubsystem") {
	imu = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
	tigerDrive = std::make_unique<TigerDrive>(imu);
	frontLeftSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_FL_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	frontRightSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_FR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	backLeftSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_BL_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	backRightSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_BR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	mecDrive = std::make_unique<frc::MecanumDrive>(*frontLeftSpark.get(), *backLeftSpark.get(), *frontRightSpark.get(), *
		backRightSpark.get());

	frontLeftSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	frontRightSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	backLeftSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	backRightSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

}

/**
 * This allows the subsystem to define a default command
 * that will run when all other commands in the subsystem
 * have stopped.
 */
void DrivebaseSubsystem::InitDefaultCommand() {
	SetDefaultCommand(new DriveCommand());
}

/**
 * Calls the tigerswerve function and passes the correct params
 * @param xAxis X Axis of joystick
 * @param yAxis Y Axis of joystick
 * @param rotAxis Rotation Axis of joystick
 * @param currentYaw Current Yaw of the robot
 */

void DrivebaseSubsystem::MecDrive(double xAxis, double yAxis, double rotAxis, double currentYaw) {
	mecDrive->DriveCartesian(xAxis, yAxis, rotAxis, -currentYaw);
}

/*
 * This is public becuase DriveCommand uses a lot of function in TigerDrive
 * @return a unique_ptr to our instance of tigerdrive
 */
const std::unique_ptr<TigerDrive>& DrivebaseSubsystem::GetTigerDrive() {
	return tigerDrive;
}

void DrivebaseSubsystem::Periodic() {
	SmartDashboard::PutNumber("IMU Yaw", tigerDrive->GetAdjYaw());
	
}
