/**
 * \file DrivebaseSubsystem.h
 * \brief This file holds all information regarding the drivebase
 * \author Drew Williams
 */

#pragma once

#include <frc/commands/Subsystem.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include "Utilities/TigerDrive/TigerDrive.h"
#include <frc/drive/MecanumDrive.h>

/**
 * This class represents a SwerveSubsystem. We hold the tigerSwerve, tigerDrive and talons in here
 */
class DrivebaseSubsystem : public frc::Subsystem {
public:
	DrivebaseSubsystem();
	void InitDefaultCommand();
	const std::unique_ptr<TigerDrive>& GetTigerDrive();
	void MecDrive(double xAxis, double yAxis, double rotAxis, double currentYaw);
	virtual void Periodic();
private:
	std::shared_ptr<AHRS> imu;
	std::unique_ptr<rev::CANSparkMax> frontLeftSpark;
	std::unique_ptr<rev::CANSparkMax> frontRightSpark;
	std::unique_ptr<rev::CANSparkMax> backLeftSpark;
	std::unique_ptr<rev::CANSparkMax> backRightSpark;

	std::unique_ptr<TigerDrive> tigerDrive;
	std::unique_ptr<frc::MecanumDrive> mecDrive;
};

