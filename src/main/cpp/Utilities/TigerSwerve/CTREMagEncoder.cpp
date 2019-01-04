#include "CTREMagEncoder.h"

CTREMagEncoder::CTREMagEncoder(const std::shared_ptr<can::TalonSRX>& talon) : m_talon(talon) {

}

CTREMagEncoder::~CTREMagEncoder() {
}

/**
 * Gets angle that the encoder is at
 * @return Rotation that the encoder is at
 */
Rotation2D CTREMagEncoder::GetRawAngle() const {
	return Rotation2D::fromRadians(GetEncoderTicks(false) / 4096.0 * 2 * M_PI);
}

/**
 * What angle are we at? WITHOUT OVERFLOW
 * @return the angle the encoder is at
 */
Rotation2D CTREMagEncoder::GetAngle() const {
	return m_offset.rotateBy(GetRawAngle());
}

/**
 * How many rotations have we gone through
 * @return number of rotations
 */
int CTREMagEncoder::GetRotations() const {
	return GetEncoderTicks(true) / 4096;
}

/**
 * Gets the quadrature encoder and than inverts it to make sure we take sensor direction into account
 * @param if we want to allow overflowing of value or not
 * @return ticks that the encoder is at
 */
int CTREMagEncoder::GetEncoderTicks(bool overflow) const {
	int ticks = m_talon->GetSelectedSensorPosition(0);
	if (!overflow) {
		ticks = ticks & 0xFFF;
	}
	return ticks;
}

/**
 * Takes in an angle and makes sure we dont overlooping math and optimization
 * @param targetAngle angle to rotate to
 * @return ticks we want to go to
 */
int CTREMagEncoder::ConvertAngleToSetpoint(Rotation2D targetAngle) {
	Rotation2D angle = m_offset.inverse().rotateBy(targetAngle);
	int ticks = ConvertAngleToEncoderTicks(angle);
	int encoderTicks = GetEncoderTicks(true);
	ticks = ticks + GetRotations() * 4096;
	int error = encoderTicks - ticks;

	if (error <= -2048) {
		ticks = ticks - 4096;
	}
	else if (error > 2048) {
		ticks = ticks + 4096;
	}
	return ticks;
}

/**
 * Utility function to convert angle to ticks
 * @param angle angle we want to convert to amount of ticks
 * @return ticks that make up angle
 */
int CTREMagEncoder::ConvertAngleToEncoderTicks(Rotation2D angle) {
	double degrees = angle.getDegrees();
	return degrees / 360.0 * 4096;
}

/**
 * sets encoders absolute sensor to ticks
 * @param ticks ticks to set to
 */
void CTREMagEncoder::SetEncoderRaw(int ticks) {
	m_talon->SetSelectedSensorPosition(ticks, 0, 0);
}
