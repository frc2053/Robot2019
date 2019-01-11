/**
 * \file TigerSwerve.h
 * \brief Controls the Swerve Drive
 * \author FRC 2481 and Drew Williams
 */

#pragma once

#include "../Math/Translation2D.h"
#include "SwerveModule.h"
#include <unordered_map>
#include "../Math/RigidTransform2D.h"

/**
 * This class represents the Swerve System as a whole. You can drive it based on joystick values
 * and tell it to brake etc..
 */
class TigerSwerve {
private:
	double xAxis = 0, yAxis = 0, rotAxis = 0, currentYaw = 0;
	std::shared_ptr<std::unordered_map<std::string, SwerveModule>> modules;
	void Drive(double xSpeed, double ySpeed, double rotSpeed, double headingOffset);
	double deg2rad(double deg);
	static void SwerveInverseKinematics(Translation2D &translation,
				double rotation, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
				Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR);
public:
	TigerSwerve(std::unordered_map<std::string, std::shared_ptr<can::TalonSRX>>& talons);
	virtual ~TigerSwerve();
	void SetBrakeMode();
	void DriveRobotOriented(double x, double y, double rotation);
	void DriveFieldOriented(double x, double y, double rotation, double gyro);
	std::shared_ptr<std::unordered_map<std::string, SwerveModule>> GetModules();
	static RigidTransform2D::Delta SwerveForwardKinematics(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle, RigidTransform2D::Delta& frVelocity,
				Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity, Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity);
};
