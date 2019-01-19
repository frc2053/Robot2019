#include "Subsystems/DrivebaseSubsystem.h"
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/commands/Command.h>
#include "Commands/Drive/DriveCommand.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include "Robot.h"
#include "Utilities/Math/RigidTransform2D.h"
#include "frc/RobotController.h"
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
	Robot::observer->SetRobotPos(RigidTransform2D(Translation2D(0,0), Rotation2D(1, 0, true)), 0.0);
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
	if(wheel.compare("fl") == 0) {
		vel = frontLeftSpark->GetEncoder().GetVelocity();
	}
	if(wheel.compare("fr") == 0) {
		vel = -frontRightSpark->GetEncoder().GetVelocity();
	}
	if(wheel.compare("bl") == 0) {
		vel = backLeftSpark->GetEncoder().GetVelocity();
	}
	if(wheel.compare("br") == 0) {
		vel = -backRightSpark->GetEncoder().GetVelocity();
	}
	double ticksPer10Ms = ConvertRPMToTicksPer10MS(vel);
	double retVal = ConvertWheelRotationsToDistance(ConvertEncoderRotationsToWheelsRotations(ConvertEncoderTicksToEncoderRotations(ticksPer10Ms)));
	return retVal;
}

Translation2D DrivebaseSubsystem::GetWheelDistance(std::string wheel) {
	double pos = 0;
	if(wheel.compare("fl") == 0) {
		pos = frontLeftSpark->GetEncoder().GetPosition();
	}
	if(wheel.compare("fr") == 0) {
		pos = -frontRightSpark->GetEncoder().GetPosition();
	}
	if(wheel.compare("bl") == 0) {
		pos = backLeftSpark->GetEncoder().GetPosition();
	}
	if(wheel.compare("br") == 0) {
		pos = -backRightSpark->GetEncoder().GetPosition();
	}
	double encoderRotations = ConvertEncoderTicksToEncoderRotations(pos);
	double wheelRotations = ConvertEncoderRotationsToWheelsRotations(encoderRotations);
	return Translation2D(ConvertWheelRotationsToDistance(wheelRotations), 0);
}

double DrivebaseSubsystem::ConvertEncoderTicksToEncoderRotations(int ticks) {
	double retVal = (double) ticks / (double) kTICKS_PER_REV_NEO;
	SmartDashboard::PutNumber("encoder rots", retVal);
	return retVal;
}

double DrivebaseSubsystem::ConvertEncoderRotationsToWheelsRotations(double rotations) {
	double output = rotations / kENCODER_REVS_PER_WHEEL_REV;
	SmartDashboard::PutNumber("wheel rots", output);
	return output;
}

double DrivebaseSubsystem::ConvertWheelRotationsToDistance(double rotations) {
	double retVal = rotations * (3.14159 * kWHEEL_DIAMETER);
	SmartDashboard::PutNumber("distance", retVal);
	return retVal;
}

double DrivebaseSubsystem::ConvertRPMToTicksPer10MS(double rpm) {
	return rpm * .008; //.008 = (1/60) * (1/1000) * (10/10) * 48
}

void DrivebaseSubsystem::Periodic() {
	SmartDashboard::PutNumber("IMU Yaw", tigerDrive->GetAdjYaw());

	if(m_first) {
		m_first = false;
		m_oldFlDistance = Robot::drivebaseSubsystem->GetWheelDistance("fl");
		m_oldFrDistance = Robot::drivebaseSubsystem->GetWheelDistance("fr");
		m_oldBlDistance = Robot::drivebaseSubsystem->GetWheelDistance("bl");
		m_oldBrDistance = Robot::drivebaseSubsystem->GetWheelDistance("br");
		m_oldGyroYaw = Rotation2D::fromDegrees(-GetTigerDrive()->GetAdjYaw());
	}

	double timestamp = RobotController::GetFPGATime();
	double deltaTimestamp = timestamp - m_oldTimestamp;
	m_oldTimestamp = timestamp;

	Translation2D newFlDistance = GetWheelDistance("fl");
	Translation2D deltaFlDistance = newFlDistance.translateBy(m_oldFlDistance.inverse());
	m_oldFlDistance = newFlDistance;

	Translation2D newFrDistance = GetWheelDistance("fr");
	Translation2D deltaFrDistance = newFrDistance.translateBy(m_oldFrDistance.inverse());
	m_oldFrDistance = newFrDistance;

	Translation2D newBlDistance = GetWheelDistance("bl");
	Translation2D deltaBlDistance = newBlDistance.translateBy(m_oldBlDistance.inverse());
	m_oldBlDistance = newBlDistance;

	Translation2D newBrDistance = GetWheelDistance("br");
	Translation2D deltaBrDistance = newBrDistance.translateBy(m_oldBrDistance.inverse());
	m_oldBrDistance = newBrDistance;

	Rotation2D newGyroYaw = Rotation2D::fromDegrees(-GetTigerDrive()->GetAdjYaw());
	Rotation2D deltaGyroYaw = newGyroYaw.rotateBy(m_oldGyroYaw.inverse());
	m_oldGyroYaw = newGyroYaw;

	RigidTransform2D::Delta deltaFlVelocity = RigidTransform2D::Delta::fromDelta(-deltaFlDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaFrVelocity = RigidTransform2D::Delta::fromDelta(-deltaFrDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaBlVelocity = RigidTransform2D::Delta::fromDelta(-deltaBlDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaBrVelocity = RigidTransform2D::Delta::fromDelta(-deltaBrDistance.getX(), 0, 0, deltaTimestamp);

	Robot::observer->UpdateRobotPoseObservation(deltaFlVelocity, deltaFrVelocity, deltaBlVelocity, deltaBrVelocity, timestamp, deltaGyroYaw);

	RigidTransform2D observerPos = Robot::observer->GetLastRobotPose();

	SmartDashboard::PutNumber("Field X", observerPos.getTranslation().getX());
	SmartDashboard::PutNumber("Field Y", observerPos.getTranslation().getY());
	SmartDashboard::PutNumber("Field Yaw", observerPos.getRotation().getDegrees());

	SmartDashboard::PutNumber("FL Drive Dist", GetWheelDistance("fl").getX());
	SmartDashboard::PutNumber("FR Drive Dist", GetWheelDistance("fr").getX());
	SmartDashboard::PutNumber("BL Drive Dist", GetWheelDistance("bl").getX());
	SmartDashboard::PutNumber("BR Drive Dist", GetWheelDistance("br").getX());

	SmartDashboard::PutNumber("FL Drive Vel", GetWheelSpeed("fl"));
	SmartDashboard::PutNumber("FR Drive Vel", GetWheelSpeed("fr"));
	SmartDashboard::PutNumber("BL Drive Vel", GetWheelSpeed("bl"));
	SmartDashboard::PutNumber("BR Drive Vel", GetWheelSpeed("br"));
}

RigidTransform2D::Delta DrivebaseSubsystem::MecanumForwardKinematics(RigidTransform2D::Delta& flVelocity, RigidTransform2D::Delta& frVelocity, RigidTransform2D::Delta& blVelocity, RigidTransform2D::Delta& brVelocity) {
	double xVelocity = (GetWheelSpeed("fl") + GetWheelSpeed("fr") + GetWheelSpeed("bl") + GetWheelSpeed("br") * (kWHEEL_DIAMETER / 2) / 4);
	double yVelocity = (-GetWheelSpeed("fl") + GetWheelSpeed("fr") + GetWheelSpeed("bl") - GetWheelSpeed("br") * (kWHEEL_DIAMETER / 2) / 4);
	double yawRate = (-GetWheelSpeed("fl") + GetWheelSpeed("fr") - GetWheelSpeed("bl") + GetWheelSpeed("br") * (kWHEEL_DIAMETER / 2) / (4 * (kWHEEL_BASE_LENGTH + kWHEEL_BASE_WIDTH)));
	SmartDashboard::PutNumber("xVel", xVelocity);
	SmartDashboard::PutNumber("yVel", yVelocity);
	return RigidTransform2D::Delta::fromDelta(xVelocity, yVelocity, yawRate, flVelocity.GetDt());
}
