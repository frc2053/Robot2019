/**
 * \file AMT10Encoder
 * \brief An object that represents our velocity encoder
 * \author Drew Williams
 */

#pragma once

#include <memory>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <ctre/phoenix/MotorControl/SensorCollection.h>
#include "../Math/Translation2D.h"

using namespace ctre::phoenix::motorcontrol;

class AMT10Encoder {
private:
	std::shared_ptr<can::TalonSRX> m_talon;
	std::string m_name;
	std::string m_calibrationKey;
	Translation2D m_offset;
public:
	AMT10Encoder(const std::shared_ptr<can::TalonSRX>& talon);
	virtual ~AMT10Encoder();

	Translation2D GetRawDistance() const;
	Translation2D GetDistance() const;
	double GetEncoderSpeed() const;
	int GetEncoderTicks() const;
	double ConvertEncoderRotationsToWheelRotations(double rotations) const;
	double ConvertWheelRotationsToEncoderRotations(double rotations) const;
	double ConvertWheelRotationsToDistance(double rotations) const;
	double ConvertDistanceToWheelRotations(double distance) const;
	int ConvertEncoderRotationsToEncoderTicks(double rotations) const;
	double ConvertEncoderTicksToEncoderRotations(int ticks) const;
	void SetEncoderTicks(int ticks, int timeout = 0);
	void ResetDistance();
};
