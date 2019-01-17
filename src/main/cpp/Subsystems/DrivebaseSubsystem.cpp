#include "Subsystems/DrivebaseSubsystem.h"
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/commands/Command.h>
#include "Commands/Drive/DriveCommand.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

#include <iostream>

/**
 * Constructor for the subsystem. We setup all the talons in here as well as the tiger drive and swerve stuff.
 */
DrivebaseSubsystem::DrivebaseSubsystem() : Subsystem("DrivebaseSubsystem") {
	imu = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
	tigerDrive = std::make_unique<TigerDrive>(imu);
	frontLeftSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_FL_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	frontRightSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_FR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	backLeftSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_BL_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	backRightSpark = std::make_unique<rev::CANSparkMax>(kDRIVESPARK_BR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	mecDrive = std::make_unique<frc::MecanumDrive>(*frontLeftSpark.get(), *backLeftSpark.get(), *frontRightSpark.get(), *
		backRightSpark.get());

	frontLeftSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	frontRightSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	backLeftSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	backRightSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

}

/**
 * This allows the subsystem to define a default command
 * that will run when all other commands in the subsystem
 * have stopped.
 */
void DrivebaseSubsystem::InitDefaultCommand() {
	SetDefaultCommand(new DriveCommand());
}

/**
 * Calls the tigerswerve function and passes the correct params
 * @param xAxis X Axis of joystick
 * @param yAxis Y Axis of joystick
 * @param rotAxis Rotation Axis of joystick
 * @param currentYaw Current Yaw of the robot
 */

void DrivebaseSubsystem::MecDrive(double xAxis, double yAxis, double rotAxis, double currentYaw) {
	mecDrive->DriveCartesian(xAxis, yAxis, rotAxis, -currentYaw);
}

/*
 * This is public becuase DriveCommand uses a lot of function in TigerDrive
 * @return a unique_ptr to our instance of tigerdrive
 */
const std::unique_ptr<TigerDrive>& DrivebaseSubsystem::GetTigerDrive() {
	return tigerDrive;
}

double DrivebaseSubsystem::GetWheelSpeed(std::string wheel) {
	double vel = 0;
	if(wheel == "fl") {
		vel = frontLeftSpark->GetVelocity();
	}
	if(wheel == "fr") {
		vel = frontRightSpark->GetVelocity();
	}
	if(wheel == "bl") {
		vel = backLeftSpark->GetVelocity();
	}
	if(wheel == "br") {
		vel = backRightSpark->GetVelocity();
	}
	return ConvertWheelRotationsToDistance(ConvertEncoderRotationsToWheelRotations(ConvertEncoderTicksToEncoderRotations(vel))) * 10;
}

Translation2D DrivebaseSubsystem::GetWheelDistance(std::string wheel) {
	double pos = 0;
	if(wheel == "fl") {
		pos = frontLeftSpark->GetPosition();
	}
	if(wheel == "fr") {
		pos = frontRightSpark->GetPosition();
	}
	if(wheel == "bl") {
		pos = backLeftSpark->GetPosition();
	}
	if(wheel == "br") {
		pos = backRightSpark->GetPosition();
	}
	double encoderRotations = ConvertEncoderTicksToEncoderRotations(pos);
	double wheelRotations = ConvertEncoderRotationsToWheelsRotations(encoderRotations);
	return Translation2D(ConvertWheelRotationsToDistance(wheelRotations), 0);
}

double ConvertEncoderTicksToEncoderRotations(int ticks) {
	return ticks / (double) kTICKS_PER_REV_OF_ENCODER;
}

double ConvertEncoderRotationsToWheelsRotations(double rotations) {
	double output = rotations / (kTICKS_PER_REV_OF_ENCODER * kDRIVE_GEAR_RATIO * kWHEEL_DIAMETER);
	return output;
}

double ConvertWheelRotationsToDistance(double rotations) {
	return rotations * (3.14159 * kWHEEL_DIAMETER);
}

void DrivebaseSubsystem::Periodic() {
	SmartDashboard::PutNumber("IMU Yaw", tigerDrive->GetAdjYaw());

	if(m_first) {
		m_first = false;
		m_oldFlDistance = Robot::drivebaseSubsystem->GetDistance("fl");
		m_oldFrDistance = Robot::drivebaseSubsystem->GetDistance("fr");
		m_oldBlDistance = Robot::drivebaseSubsystem->GetDistance("bl");
		m_oldBrDistance = Robot::drivebaseSubsystem->GetDistance("br");
		m_oldGyroYaw = Rotation2D::fromDegrees(-GetTigerDrive()->GetAdjYaw());
	}
}

RigidTransform2D::Delta DrivebaseSubsystem::MecanumForwardKinematics(RigidTransform2D::Delta& flVelocity, RigidTransform2D::Delta& frVelocity, RigidTransform2D::Delta& blVelocity, RigidTransform2D::Delta& brVelocity {
	double xVelocity = (GetWheelSpeed("fl") + GetWheelSpeed("fr") + GetWheelSpeed("bl") + GetWheelSpeed("br") * (kWHEEL_DIAMTER / 2) / 4);
	double yVelocity = (-GetWheelSpeed("fl") + GetWheelSpeed("fr") + GetWheelSpeed("bl") - GetWheelSpeed("br") * (kWHEEL_DIAMTER / 2) / 4);
	double yawRate = (-GetWheelSpeed("fl") + GetWheelSpeed("fr") - GetWheelSpeed("bl") + GetWheelSpeed("br") * (kWHEEL_DIAMTER / 2) / (4 * (kWHEEL_BASE_LENGTH + kWHEEL_BASE_WIDTH)));
	return RigidTransform2D::Delta::fromDelta(xVelocity, yVelocity, yawRate, flVelocity.GetDt());
}
