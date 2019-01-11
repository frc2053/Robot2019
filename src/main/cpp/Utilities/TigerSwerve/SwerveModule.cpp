#include "Utilities/TigerSwerve/SwerveModule.h"

/**
 * Constructor for a swerve module.
 * @param driveController The pointer to a Talon SRX that controls the driving wheel
 * @param rotateController The pointer to a Talon SRX that controls the rotation motor
 */
SwerveModule::SwerveModule(std::shared_ptr<can::TalonSRX> driveController, std::shared_ptr<can::TalonSRX> rotateController) {
	_driveController.reset(driveController.get());
	_rotateController.reset(rotateController.get());
	_angleEncoder = std::make_shared<CTREMagEncoder>(rotateController);
	_driveEncoder = std::make_shared<AMT10Encoder>(driveController);
	isOptimizedAngle = false;
}

/**
 * Default Destructor
 */
SwerveModule::~SwerveModule() {

}

/**
 * Returns angle that the wheel is currentYawat
 * @return angle in radians that the wheel is at
 */
Rotation2D SwerveModule::GetAngle() const {
	return _angleEncoder->GetAngle();
}

/**
 * Returns the velocity of the drive wheel in native units
 * (Sensor Units per 100ms)
 * @return velocity of the drive wheel
 */
double SwerveModule::GetVelocity() const {
	return _driveEncoder->GetEncoderSpeed();
}

Translation2D SwerveModule::GetDistance() const {
	return _driveEncoder->GetDistance();
}

/**
 *	This controls the motor setpoint of where to send the rotation motor
 *
 * @param angle the angle we need to rotate to in radians
 * @param doOptimization do we want to make sure we always rotate less than 90 degrees?
 */
void SwerveModule::SetAngle(Rotation2D angle, bool doOptimization) {
	//gets the current angle of the wheel
	Rotation2D currentAngle = _angleEncoder->GetAngle();
	//Difference between the current angle and angle we want to go to
	Rotation2D deltaAngle = currentAngle.rotateBy(angle.inverse());
	//make sure we get abs value
	deltaAngle = Rotation2D::fromDegrees(fabs(deltaAngle.getDegrees()));

	if(doOptimization) {
		//if we have to rotate more than 90 degrees and rotating less than 270 degrees
		if(deltaAngle.getRadians() > M_PI_2 && deltaAngle.getRadians() <= 3 * M_PI_2) {
			//rotate to "equivalent angle"
			angle = angle.rotateBy(Rotation2D::fromRadians(M_PI));
			isOptimizedAngle = true;
		}
		else {
			isOptimizedAngle = false;
		}
	}
	//convert angle to ticks
	int setpoint = _angleEncoder->ConvertAngleToSetpoint(angle);
	_rotateController->Set(ControlMode::Position, setpoint);
}

/**
 * STOP DRIVE MOTOR
 */
void SwerveModule::Stop() {
	_driveController->Set(ControlMode::PercentOutput, 0);
}

/**
 * This is what we call from the SwerveDrive class to set the speed and angle of each module.
 * @param speed The speed to set in PercentOutput
 * @param angle The angle to set in Rotation2D
 * @param doOptimization do we want to make sure we always rotate 90 degrees or less
 */
void SwerveModule::Set(double speed, Rotation2D angle, bool doOptimization) {
	SetAngle(angle, doOptimization);
	if(isOptimizedAngle) {
		_driveController->Set(ControlMode::PercentOutput, speed*-1);
	}
	else {
		_driveController->Set(ControlMode::PercentOutput, speed);
	}
}
