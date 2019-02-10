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
#include <frc/shuffleboard/Shuffleboard.h>

/**
 * Constructor for the subsystem. We setup all the talons in here as well as the tiger drive and swerve stuff.
 */
DrivebaseSubsystem::DrivebaseSubsystem() : Subsystem("DrivebaseSubsystem") {
	imu = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
	tigerDrive = std::make_unique<TigerDrive>(imu);
	frontLeftSpark = std::make_unique<rev::CANSparkMax>(Robot::robotMap->kDRIVESPARK_FL_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	frontRightSpark = std::make_unique<rev::CANSparkMax>(Robot::robotMap->kDRIVESPARK_FR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	backLeftSpark = std::make_unique<rev::CANSparkMax>(Robot::robotMap->kDRIVESPARK_BL_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	backRightSpark = std::make_unique<rev::CANSparkMax>(Robot::robotMap->kDRIVESPARK_BR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
	mecDrive = std::make_unique<frc::MecanumDrive>(*frontLeftSpark.get(), *backLeftSpark.get(), *frontRightSpark.get(), *
		backRightSpark.get());
	mecDrive->SetSafetyEnabled(false);
	frontLeftSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	frontRightSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	backLeftSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	backRightSpark->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	Robot::observer->SetRobotPos(RigidTransform2D(Translation2D(0,0), Rotation2D(1, 0, true)), 0.0);

	frontLeftSpark->SetRampRate(.5);
	frontRightSpark->SetRampRate(.5);
	backLeftSpark->SetRampRate(.5);
	backRightSpark->SetRampRate(.5);

	flPos = 0;
	frPos = 0;
	blPos = 0;
	brPos = 0;
	m_first = true;
	m_driveController = new DriveController(Robot::observer);
	pose = new RobotPose();
	Shuffleboard::GetTab("Field Position").Add("Robot Pose", pose);

	flVelPID = std::make_unique<rev::CANPIDController>(*frontLeftSpark.get());
	frVelPID = std::make_unique<rev::CANPIDController>(*frontRightSpark.get());
	blVelPID = std::make_unique<rev::CANPIDController>(*backLeftSpark.get());
	brVelPID = std::make_unique<rev::CANPIDController>(*backRightSpark.get());

	ConfigSparkVelocityPID();
}

DriveController* DrivebaseSubsystem::GetDriveController() {
	return m_driveController;
}

/**
 * This allows the subsystem to define a default command
 * that will run when all other commands in the subsystem
 * have stopped.
 */
void DrivebaseSubsystem::InitDefaultCommand() {
	//SetDefaultCommand(new DriveCommand());
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

void DrivebaseSubsystem::ConfigSparkVelocityPID() {
	flVelPID->SetP(Robot::robotMap->kAUTO_CONTROLLER_P);
	flVelPID->SetI(Robot::robotMap->kAUTO_CONTROLLER_I);
	flVelPID->SetD(Robot::robotMap->kAUTO_CONTROLLER_D);
	flVelPID->SetFF(Robot::robotMap->kAUTO_CONTROLLER_V);
	flVelPID->SetIZone(1e-6);
	flVelPID->SetOutputRange(-1, 1);

	frVelPID->SetP(Robot::robotMap->kAUTO_CONTROLLER_P);
	frVelPID->SetI(Robot::robotMap->kAUTO_CONTROLLER_I);
	frVelPID->SetD(Robot::robotMap->kAUTO_CONTROLLER_D);
	frVelPID->SetFF(Robot::robotMap->kAUTO_CONTROLLER_V);
	frVelPID->SetIZone(1e-6);
	frVelPID->SetOutputRange(-1, 1);

	blVelPID->SetP(Robot::robotMap->kAUTO_CONTROLLER_P);
	blVelPID->SetI(Robot::robotMap->kAUTO_CONTROLLER_I);
	blVelPID->SetD(Robot::robotMap->kAUTO_CONTROLLER_D);
	blVelPID->SetFF(Robot::robotMap->kAUTO_CONTROLLER_V);
	blVelPID->SetIZone(1e-6);
	blVelPID->SetOutputRange(-1, 1);

	brVelPID->SetP(Robot::robotMap->kAUTO_CONTROLLER_P);
	brVelPID->SetI(Robot::robotMap->kAUTO_CONTROLLER_I);
	brVelPID->SetD(Robot::robotMap->kAUTO_CONTROLLER_D);
	brVelPID->SetFF(Robot::robotMap->kAUTO_CONTROLLER_V);
	brVelPID->SetIZone(1e-6);
	brVelPID->SetOutputRange(-1, 1);
}

void DrivebaseSubsystem::SetWheelVelocities(double fl, double fr, double bl, double br) {
	flVelPID->SetReference(ConvertInPerSecondToRPM(fl), rev::ControlType::kVelocity);
	frVelPID->SetReference(ConvertInPerSecondToRPM(fr), rev::ControlType::kVelocity);
	blVelPID->SetReference(ConvertInPerSecondToRPM(bl), rev::ControlType::kVelocity);
	brVelPID->SetReference(ConvertInPerSecondToRPM(br), rev::ControlType::kVelocity);
}

double DrivebaseSubsystem::ConvertInPerSecondToRPM(double ips) {
	double rpm = 0;

	rpm = ips / (M_PI * 6);

	return rpm;
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
	double ticksPer100Ms = ConvertRPMToTicksPer100MS(vel);
	double retVal = ConvertWheelRotationsToDistance(ConvertEncoderRotationsToWheelsRotations(ConvertEncoderTicksToEncoderRotations(ticksPer100Ms))) * 10;
	return retVal;
}

Translation2D DrivebaseSubsystem::GetWheelDistance(std::string wheel) {
	double pos = 0;
	if(wheel.compare("fl") == 0) {
		pos = frontLeftSpark->GetEncoder().GetPosition() - flPos;
	}
	if(wheel.compare("fr") == 0) {
		pos = -frontRightSpark->GetEncoder().GetPosition() - frPos;
	}
	if(wheel.compare("bl") == 0) {
		pos = backLeftSpark->GetEncoder().GetPosition() - blPos;
	}
	if(wheel.compare("br") == 0) {
		pos = -backRightSpark->GetEncoder().GetPosition() - brPos;
	}
	double wheelRotations = ConvertEncoderRotationsToWheelsRotations(pos);
	return Translation2D(ConvertWheelRotationsToDistance(wheelRotations), 0);
}

double DrivebaseSubsystem::ConvertEncoderTicksToEncoderRotations(int ticks) {
	double retVal = (double) ticks / (double) Robot::robotMap->kTICKS_PER_REV_NEO;
	return retVal;
}

double DrivebaseSubsystem::ConvertEncoderRotationsToWheelsRotations(double rotations) {
	double output = rotations / Robot::robotMap->kENCODER_REVS_PER_WHEEL_REV;
	return output;
}

double DrivebaseSubsystem::ConvertWheelRotationsToDistance(double rotations) {
	double retVal = rotations * (3.14159 * Robot::robotMap->kWHEEL_DIAMETER);
	return retVal;
}

double DrivebaseSubsystem::ConvertRPMToTicksPer100MS(double rpm) {
	return rpm * .08; //.08 = (1/60) * (1/1000) * (100/1) * 48
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

	pose->SetX(observerPos.getTranslation().getX());
	pose->SetY(observerPos.getTranslation().getY());
	pose->SetHeading(observerPos.getRotation().getDegrees());

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
	double xVelocity = (flVelocity.GetX() + frVelocity.GetX()  + blVelocity.GetX()  + brVelocity.GetX()) * ((Robot::robotMap->kWHEEL_DIAMETER / 2) / (4 * M_PI));
	double yVelocity = (-flVelocity.GetX() + frVelocity.GetX() + blVelocity.GetX() - brVelocity.GetX()) * ((Robot::robotMap->kWHEEL_DIAMETER / 2) / (4 * M_PI));
	double yawRate = (-flVelocity.GetX() + frVelocity.GetX() - blVelocity.GetX() + brVelocity.GetX()) * ((Robot::robotMap->kWHEEL_DIAMETER / 2) / (4 * (Robot::robotMap->kWHEEL_BASE_LENGTH + Robot::robotMap->kWHEEL_BASE_WIDTH)));
	return RigidTransform2D::Delta::fromDelta(yVelocity, xVelocity, yawRate, flVelocity.GetDt());
}

void DrivebaseSubsystem::ZeroEncoders() {
	flPos = frontLeftSpark->GetEncoder().GetPosition();
	frPos = -frontRightSpark->GetEncoder().GetPosition();
	blPos = backLeftSpark->GetEncoder().GetPosition();
	brPos = -backRightSpark->GetEncoder().GetPosition();
}