/**
 * \file TigerDrive.h
 * \brief Controls the auto rotation of the robot
 * \author Drew Williams & Douglas Williams
 */

#pragma once

#include <AHRS.h>
#include <frc/PIDController.h>
#include <frc/PIDOutput.h>

/**
 * This class lets us control the rotation of the robot in teleop
 * Holds the PID Controller and IMU
 */
class TigerDrive : public frc::PIDOutput
{
private:
	std::unique_ptr<frc::PIDController> rotateController;
	std::shared_ptr<AHRS> imu;

	bool tooFarCW;
	bool tooFarCCW;
	double rotateToAngleRate;
	double yawOffset;
	bool isRotDone;
	bool controllerOverride;

	void ConfigureIMU();
	double CalculateSpeedAndOvershoot(double speedMulti);
	float GetImuYaw();
public:
	TigerDrive(const std::shared_ptr<AHRS>& imu);
	virtual ~TigerDrive();
	void ZeroYaw();
	double GetAdjYaw();
	bool GetIsRotDone();
	bool GetIsRotDoneOverride();
	void SetIsRotDoneOverride(bool isDone);
	void SetIsRotDone(bool isDone);
	void SetAdjYaw(double offset);

	void SetAngleTarget(double angle);
	double CalculateRotationValue(double angleToRotateTo, double speedMultiplier);

	void PIDWrite(double output) {
	    rotateToAngleRate = output;
	}
};
