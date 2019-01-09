#include "Utilities/Tigerdrive/TigerDrive.h"
#include "RobotMap.h"

/**
 * Constructor which sets up the PID Controller and IMU.
 * @param imuP shared_ptr to the IMU pointer (owned by drivebase subsystem)
 */
TigerDrive::TigerDrive(const std::shared_ptr<AHRS>& imuP)
{
	rotateController = std::make_unique<frc::PIDController>(YAW_CONTROLLER_P, YAW_CONTROLLER_I, YAW_CONTROLLER_D, imu.get(), this);
	imu = imuP;

	ConfigureIMU();

	tooFarCW = false;
	tooFarCCW = false;
	isRotDone = false;
	controllerOverride = false;
	rotateToAngleRate = 1;
	yawOffset = 0;
}

/**
 * Default destructor
 */
TigerDrive::~TigerDrive() {

}

/**
 * Sets the input and output range and tolerance of the PID controller
 */
void TigerDrive::ConfigureIMU() {
	rotateController->SetInputRange(-180.0, 180.0);
	rotateController->SetOutputRange(-1.0, 1.0);
	rotateController->SetAbsoluteTolerance(YAW_CONTROLLER_TOLERANCE);
	rotateController->SetContinuous(true);
}

/**
 * Sets the yaw on the IMU to zero
 */
void TigerDrive::ZeroYaw() {
	imu->ZeroYaw();
}

/**
 * Calculates the rotation rate to rotate the robot at
 * @param angleToRotateTo angle to rotate to in degrees
 * @param speedMultiplier how fast we rotate
 * @return rotation rate
 */
double TigerDrive::CalculateRotationValue(double angleToRotateTo, double speedMultiplier) {
	double speed = 0;
	if(!controllerOverride) {
		speed = CalculateSpeedAndOvershoot(speedMultiplier);
	}
	else {
		speed = 0;
	}
	return speed;
}

/**
 * Calculates the speed and overshoot using the PID Controller
 * @param speedMulti
 * @return the rotation rate
 */
double TigerDrive::CalculateSpeedAndOvershoot(double speedMulti) {
	double calculatedRotate = 0;
	if(tooFarCW || tooFarCCW)
	{
		rotateController->Enable();
		isRotDone = false;
		calculatedRotate = rotateToAngleRate * speedMulti;
	}
	return calculatedRotate;
}

/**
 * Calculates the yaw of the robot based on the offset we passed in.
 * Usually used if we dont start straight forward in auto.
 * @return the "real" yaw of the robot
 */
double TigerDrive::GetAdjYaw() {
	double imuRaw = imu->GetYaw();
	double calculatedOffset = imuRaw + yawOffset;
	if(calculatedOffset >= 180)
	{
		calculatedOffset = calculatedOffset - 360;
	}
	return calculatedOffset;
}

void TigerDrive::SetAngleTarget(double angle) {
	rotateController->SetSetpoint(angle);
}

/**
 * Sets the adjusted yaw so we can set this in auto modes to make sure we rotate correctly.
 * @param offset
 */
void TigerDrive::SetAdjYaw(double offset)
{
	yawOffset = offset;
}

/**
 * Sets if we are done rotating
 * @param isDone are we done?
 */
void TigerDrive::SetIsRotDone(bool isDone)
{
	isRotDone = isDone;
}

/**
 * Have we overrode our PID Controller with the controller
 * @param isDone have we moved the rotation axis
 */
void TigerDrive::SetIsRotDoneOverride(bool isDone)
{
	controllerOverride = isDone;
}

/**
 * Gets the yaw of the imu.
 * @return the "raw" imu yaw
 */
float TigerDrive::GetImuYaw()
{
	return imu->GetYaw();
}

/**
 * Are we done rotating?
 * @return boolean that says if we are done rotating?
 */
bool TigerDrive::GetIsRotDone()
{
	return isRotDone;
}

/**
 * Gets if we have overridden the pid controller
 * @return boolean that says if we have overridden the PID
 */
bool TigerDrive::GetIsRotDoneOverride()
{
	return controllerOverride;
}
