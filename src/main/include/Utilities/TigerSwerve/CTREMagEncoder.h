/**
 * \file CTREMagEncoder.h
 * \brief This holds encoder information for a swerve drive
 * \author FRC 2481
 */

#pragma once

#include <memory>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <ctre/phoenix/MotorControl/SensorCollection.h>
#include "../Math/Rotation2D.h"

using namespace ctre::phoenix::motorcontrol;

/**
 * This class represents an encoder that is attached to a
 * swerve module for rotation. We can do fancy math in this class
 */
class CTREMagEncoder {
private:
	std::shared_ptr<can::TalonSRX> m_talon;
	std::string m_name;
	std::string m_calibrationKey;
	Rotation2D m_offset;
public:
	CTREMagEncoder(const std::shared_ptr<can::TalonSRX>& talon);
	virtual ~CTREMagEncoder();
	Rotation2D GetRawAngle() const;
	Rotation2D GetAngle() const;
	int GetRotations() const;
	int GetEncoderTicks(bool overflow = false) const;
	void Calibrate();
	int ConvertAngleToSetpoint(Rotation2D targetAngle);
	int ConvertAngleToEncoderTicks(Rotation2D angle);
	void SetEncoderRaw(int ticks);
};
