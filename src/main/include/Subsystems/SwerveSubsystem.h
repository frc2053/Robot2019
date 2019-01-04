/**
 * \file SwerveSubsystem.h
 * \brief This file holds all information regarding the drivebase
 * \author Drew Williams
 */

#pragma once

#include <Commands/Subsystem.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include "../Utilities/TigerSwerve/TigerSwerve.h"
#include "../Utilities/TigerDrive/TigerDrive.h"
#include "ObserverSubsystem.h"
#include "../Utilities/TigerSwerve/DriveController.h"


using namespace ctre::phoenix::motorcontrol;

/**
 * This class represents a SwerveSubsystem. We hold the tigerSwerve, tigerDrive and talons in here
 */
class SwerveSubsystem : public frc::Subsystem {
public:
	SwerveSubsystem();
	void InitDefaultCommand();
	const std::unique_ptr<TigerDrive>& GetTigerDrive();
	const std::unique_ptr<TigerSwerve>& GetTigerSwerve();
	const std::unique_ptr<DriveController>& GetDriveController();
	void SwerveDrive(double xAxis, double yAxis, double rotAxis, double currentYaw);
	void CalibrateWheels();
	virtual void Periodic();
	void ResetRobotPose(RigidTransform2D pose);
	const std::shared_ptr<ObserverSubsystem>& GetObserver();
private:
	std::unordered_map<std::string, std::shared_ptr<can::TalonSRX>> talons;
	std::shared_ptr<AHRS> imu;

	std::unique_ptr<TigerSwerve> tigerSwerve;
	std::unique_ptr<TigerDrive> tigerDrive;
	std::shared_ptr<ObserverSubsystem> observer;
	std::unique_ptr<DriveController> driveController;

	void ConfigureRotationMotors();
	void ConfigureDriveMotors();
	int AbsMod(int value, int ticks);
	int OptimizeRot(int value, int ticks);
	int absVal = 0;
	int halfTicks = 0;
	double m_xVel, m_yVel, m_yawRate;
	double m_oldTimestamp;
	bool m_first;
	Rotation2D m_oldFlAngle, m_oldFrAngle, m_oldBlAngle, m_oldBrAngle;
	Translation2D m_oldFlDistance, m_oldFrDistance, m_oldBlDistance, m_oldBrDistance;
	Rotation2D m_oldGyroYaw;
	Translation2D m_motionSetpoint;
};

