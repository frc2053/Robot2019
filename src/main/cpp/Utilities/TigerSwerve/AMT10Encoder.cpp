#include "Utilities/TigerSwerve/AMT10Encoder.h"
#include "RobotMap.h"

/**
 * Constructor
 * @param talon const ref to our drive Talon
 */
AMT10Encoder::AMT10Encoder(const std::shared_ptr<can::TalonSRX>& talon) : m_talon(talon) {

}

/**
 * Destructor
 */
AMT10Encoder::~AMT10Encoder() {

}

/**
 * Gets the distance we have gone in our unit of choice
 * @return Translation where the X component is our distance
 */
Translation2D AMT10Encoder::GetRawDistance() const {
	double encoderRotations = ConvertEncoderTicksToEncoderRotations(GetEncoderTicks());
	double wheelRotations = ConvertEncoderRotationsToWheelRotations(encoderRotations);
	return Translation2D(ConvertWheelRotationsToDistance(wheelRotations), 0);
}

/**
 * Gets our distance we have gone in our units
 * @return Translation2D where X component is our distance
 */
Translation2D AMT10Encoder::GetDistance() const {
	return GetRawDistance().translateBy(m_offset.inverse());
}

/**
 * Gets encoder reading from talon
 * @return int ticks from talon
 */
int AMT10Encoder::GetEncoderTicks() const {
	int ticks = m_talon->GetSelectedSensorPosition(0);
	return ticks;
}

/**
 * Sets the encoder ticks we have gone on the talon
 * @param ticks tick setpoint
 * @param timeout 0 if we dont care about error messages, 10 is default
 */
void AMT10Encoder::SetEncoderTicks(int ticks, int timeout) {
	m_talon->SetSelectedSensorPosition(ticks, 0, timeout);
}

/**
 * Gets our encoder velocity from talon and converts to distance / time
 * @return Encoder Velocity in our unit / time
 */
double AMT10Encoder::GetEncoderSpeed() const {
	//not sure why we are multiplying by 10...
	return ConvertWheelRotationsToDistance(ConvertEncoderRotationsToWheelRotations(ConvertEncoderTicksToEncoderRotations(m_talon->GetSelectedSensorVelocity(0)))) * 10;
}

double AMT10Encoder::ConvertEncoderRotationsToWheelRotations(double rotations) const {
	double output;
	output = rotations / kENCODERREVPERWHEELREV;
	return output;
}

double AMT10Encoder::ConvertWheelRotationsToEncoderRotations(double rotations) const {
	double output;
	output = rotations * kENCODERREVPERWHEELREV;
	return output;
}

double AMT10Encoder::ConvertWheelRotationsToDistance(double rotations) const {
	return rotations * kINCHESPERWEELREV;
}

double AMT10Encoder::ConvertDistanceToWheelRotations(double distance) const {
	return distance / kINCHESPERWEELREV;
}

int AMT10Encoder::ConvertEncoderRotationsToEncoderTicks(double rotations) const {
	return rotations * kTICKSPERREVOFENCODER;
}

double AMT10Encoder::ConvertEncoderTicksToEncoderRotations(int ticks) const {
	return ticks / ((double) kTICKSPERREVOFENCODER);
}

/**
 * Sets offset to the raw distance of the encoder
 */
void AMT10Encoder::ResetDistance() {
	m_offset = GetRawDistance();
}
