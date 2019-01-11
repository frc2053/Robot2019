/**
 * \file SwerveModule.h
 * \brief This is a swerve module class.
 * \author FRC 2481 & Drew Williams
 */

#pragma once

#include "CTREMagEncoder.h"
#include <math.h>
#include "../Math/Translation2D.h"
#include "../Math/Rotation2D.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "../TigerSwerve/AMT10Encoder.h"

using namespace ctre::phoenix::motorcontrol;

/**
 * This class represents a Swerve Module. This is one of the wheel saddles that
 * hold the drive and rotation motors and assembly.
 */
class SwerveModule {
public:
	SwerveModule(std::shared_ptr<can::TalonSRX> driveController, std::shared_ptr<can::TalonSRX> rotateController);
	virtual ~SwerveModule();

	Rotation2D GetAngle() const;
	void SetAngle(Rotation2D angle, bool doOptimization);
	void Set(double speed, Rotation2D angle, bool doOptimization);
	void Stop();
	double GetVelocity() const;
	Translation2D GetDistance() const;
private:
	std::shared_ptr<can::TalonSRX> _driveController;
	std::shared_ptr<can::TalonSRX> _rotateController;
	std::shared_ptr<CTREMagEncoder> _angleEncoder;
	std::shared_ptr<AMT10Encoder> _driveEncoder;
	bool isOptimizedAngle;
};
