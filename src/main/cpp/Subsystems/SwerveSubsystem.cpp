#include "Subsystems/SwerveSubsystem.h"
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Commands/Drive/ZeroWheels.h"
#include <frc/commands/Command.h>
#include <frc/RobotController.h>
#include "Commands/Drive/DriveCommand.h"
#include <iostream>

/**
 * Constructor for the subsystem. We setup all the talons in here as well as the tiger drive and swerve stuff.
 */
SwerveSubsystem::SwerveSubsystem() : Subsystem("SwerveSubsystem") {

	talons.insert(std::make_pair(FLDRIVEID, std::make_unique<can::TalonSRX>(kDRIVETALON_FL_DRIVE_ID)));
	talons.insert(std::make_pair(FRDRIVEID, std::make_unique<can::TalonSRX>(kDRIVETALON_FR_DRIVE_ID)));
	talons.insert(std::make_pair(BLDRIVEID, std::make_unique<can::TalonSRX>(kDRIVETALON_BL_DRIVE_ID)));
	talons.insert(std::make_pair(BRDRIVEID, std::make_unique<can::TalonSRX>(kDRIVETALON_BR_DRIVE_ID)));

	talons.insert(std::make_pair(FLROTID, std::make_unique<can::TalonSRX>(kDRIVETALON_FL_ROT_ID)));
	talons.insert(std::make_pair(FRROTID, std::make_unique<can::TalonSRX>(kDRIVETALON_FR_ROT_ID)));
	talons.insert(std::make_pair(BLROTID, std::make_unique<can::TalonSRX>(kDRIVETALON_BL_ROT_ID)));
	talons.insert(std::make_pair(BRROTID, std::make_unique<can::TalonSRX>(kDRIVETALON_BR_ROT_ID)));

	m_xVel = 0;
	m_yVel = 0;
	m_yawRate = 0;
	m_oldTimestamp = 1;
	m_first = true;

	m_oldFlAngle = Rotation2D(1, 0, true);
	m_oldFrAngle = Rotation2D(1, 0, true);
	m_oldBlAngle = Rotation2D(1, 0, true);
	m_oldBrAngle = Rotation2D(1, 0, true);

	m_oldFlDistance = Translation2D(0, 0);
	m_oldFrDistance = Translation2D(0, 0);
	m_oldBlDistance = Translation2D(0, 0);
	m_oldBrDistance = Translation2D(0, 0);

	m_oldGyroYaw = Rotation2D(1, 0, true);

	observer = std::make_shared<ObserverSubsystem>();
	observer->SetRobotPos(RigidTransform2D(Translation2D(0, 0), Rotation2D(1, 0, true)), 0.0);
	imu = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
	tigerSwerve = std::make_unique<TigerSwerve>(talons);
	tigerDrive = std::make_unique<TigerDrive>(imu);
	driveController = std::make_unique<DriveController>(observer);

	ConfigureRotationMotors();
	ConfigureDriveMotors();

	SmartDashboard::PutData("Zero Wheels", new ZeroWheels());
}

const std::shared_ptr<ObserverSubsystem>& SwerveSubsystem::GetObserver() {
	return observer;
}

/**
 * Configures the rotation motors to the correct parameters
 */
void SwerveSubsystem::ConfigureRotationMotors() {
	for(auto talonToConfigure : talons) {
		//if we have a rotation motor
		if(talonToConfigure.first.find("Rotation") != std::string::npos) {
			talonToConfigure.second->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
			talonToConfigure.second->SetSensorPhase(kSWERVE_ROT_PHASE);
			talonToConfigure.second->SetInverted(kSWERVE_ROT_INVERT);
			talonToConfigure.second->Config_kP(0, kSWERVE_ROT_P, 10);
			talonToConfigure.second->Config_kI(0, kSWERVE_ROT_I, 10);
			talonToConfigure.second->Config_kD(0, kSWERVE_ROT_D, 10);
			talonToConfigure.second->ConfigPeakOutputForward(kSWERVE_ROT_PEAKOUTPUT, 10);
			talonToConfigure.second->ConfigPeakOutputReverse(-kSWERVE_ROT_PEAKOUTPUT, 10);
			talonToConfigure.second->ConfigAllowableClosedloopError(0, kSWERVE_CLOSED_LOOP_TOLERANCE, 10);
			talonToConfigure.second->ConfigContinuousCurrentLimit(kSWERVE_CONT_AMPS, 10);
			talonToConfigure.second->ConfigPeakCurrentLimit(kSWERVE_PEAK_AMPS, 10);
			talonToConfigure.second->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
		}
	}
}

/**
 * Configures the drive motors to the correct parameters
 */
void SwerveSubsystem::ConfigureDriveMotors() {
	for(auto talonToConfigure : talons) {
		if(talonToConfigure.first.find("Drive") != std::string::npos) {
			talonToConfigure.second->SetInverted(kSWERVE_DRIVE_INVERT);
			talonToConfigure.second->GetSensorCollection().SetQuadraturePosition(0, 10);
			talonToConfigure.second->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
			talonToConfigure.second->SetSensorPhase(kSWERVE_DRIVE_PHASE);
			talonToConfigure.second->ConfigOpenloopRamp(kSWERVE_DRIVE_OPENLOOP_RAMP, 10);
			talonToConfigure.second->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
		}
	}
}

/**
 * Calibrates the wheels. This moves the wheels to the calibration position!
 */
void SwerveSubsystem::CalibrateWheels() {

	for(auto talonToConfigure : talons) {
		//if we have a rotation motor
		if(talonToConfigure.first.find("Rotation") != std::string::npos) {
			talonToConfigure.second->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
		}
	}

	int flpw = talons[FLROTID]->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int frpw = talons[FRROTID]->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int blpw = talons[BLROTID]->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;
	int brpw = talons[BRROTID]->GetSensorCollection().GetPulseWidthPosition() & 0xFFF;

	int flset = flpw - kDRIVEBASE_FL_CAL;
	int frset = frpw - kDRIVEBASE_FR_CAL;
	int blset = blpw - kDRIVEBASE_BL_CAL;
	int brset = brpw - kDRIVEBASE_BR_CAL;

	talons[FLROTID]->Set(ControlMode::Position, flset);
	talons[FRROTID]->Set(ControlMode::Position, frset);
	talons[BLROTID]->Set(ControlMode::Position, blset);
	talons[BRROTID]->Set(ControlMode::Position, brset);
}

int SwerveSubsystem::AbsMod(int value, int ticks) {
	int retVal = value;
	if(value < 0) {
		retVal = ticks - abs(value);
	}
	return retVal;
}

/**
 * This allows the subsystem to define a default command
 * that will run when all other commands in the subsystem
 * have stopped.
 */
void SwerveSubsystem::InitDefaultCommand() {
	SetDefaultCommand(new DriveCommand());
}

/**
 * Calls the tigerswerve function and passes the correct params
 * @param xAxis X Axis of joystick
 * @param yAxis Y Axis of joystick
 * @param rotAxis Rotation Axis of joystick
 * @param currentYaw Current Yaw of the robot
 */
void SwerveSubsystem::SwerveDrive(double xAxis, double yAxis, double rotAxis, double currentYaw) {
	tigerSwerve->DriveFieldOriented(xAxis, yAxis, rotAxis, currentYaw);
}

int SwerveSubsystem::OptimizeRot(int value, int ticks) {
	int retVal = value;

	absVal = abs(value);
	halfTicks =  ticks/2;

	if(absVal > halfTicks) {
		retVal = (ticks - absVal);
	}

	if (value > halfTicks) {
		retVal = retVal * -1;
	}

	return retVal;
}

/**
 * This is public becuase DriveCommand uses a lot of function in TigerDrive
 * @return a unique_ptr to our instance of tigerdrive
 */
const std::unique_ptr<TigerDrive>& SwerveSubsystem::GetTigerDrive() {
	return tigerDrive;
}

const std::unique_ptr<TigerSwerve>& SwerveSubsystem::GetTigerSwerve() {
	return tigerSwerve;
}

const std::unique_ptr<DriveController>& SwerveSubsystem::GetDriveController() {
	return driveController;
}

void SwerveSubsystem::ResetRobotPose(RigidTransform2D pose) {
	m_first = true;
	observer->ResetPose(pose);
}

void SwerveSubsystem::Periodic() {

	if(m_first) {
		m_first = false;
		m_oldFlAngle = GetTigerSwerve()->GetModules()->at(FLMODULEID).GetAngle();
		m_oldFrAngle = GetTigerSwerve()->GetModules()->at(FRMODULEID).GetAngle();
		m_oldBlAngle = GetTigerSwerve()->GetModules()->at(BLMODULEID).GetAngle();
		m_oldBrAngle = GetTigerSwerve()->GetModules()->at(BRMODULEID).GetAngle();
		m_oldFlDistance = GetTigerSwerve()->GetModules()->at(FLMODULEID).GetDistance();
		m_oldFrDistance = GetTigerSwerve()->GetModules()->at(FRMODULEID).GetDistance();
		m_oldBlDistance = GetTigerSwerve()->GetModules()->at(BLMODULEID).GetDistance();
		m_oldBrDistance = GetTigerSwerve()->GetModules()->at(BRMODULEID).GetDistance();
		m_oldGyroYaw = Rotation2D::fromDegrees(-GetTigerDrive()->GetAdjYaw());
	}

	double timestamp = RobotController::GetFPGATime();
	double deltaTimestamp = timestamp - m_oldTimestamp;
	m_oldTimestamp = timestamp;

	Rotation2D newFlAngle = GetTigerSwerve()->GetModules()->at(FLMODULEID).GetAngle();
	Rotation2D deltaFlAngle = newFlAngle.rotateBy(m_oldFlAngle.inverse());
	m_oldFlAngle = newFlAngle;

	Translation2D newFlDistance = GetTigerSwerve()->GetModules()->at(FLMODULEID).GetDistance();
	Translation2D deltaFlDistance = newFlDistance.translateBy(m_oldFlDistance.inverse());
	m_oldFlDistance = newFlDistance;

	Rotation2D newFrAngle = GetTigerSwerve()->GetModules()->at(FRMODULEID).GetAngle();
	Rotation2D deltaFrAngle = newFrAngle.rotateBy(m_oldFrAngle.inverse());
	m_oldFrAngle = newFrAngle;

	Translation2D newFrDistance = GetTigerSwerve()->GetModules()->at(FRMODULEID).GetDistance();
	Translation2D deltaFrDistance = newFrDistance.translateBy(m_oldFrDistance.inverse());
	m_oldFrDistance = newFrDistance;

	Rotation2D newBlAngle = GetTigerSwerve()->GetModules()->at(BLMODULEID).GetAngle();
	Rotation2D deltaBlAngle = newBlAngle.rotateBy(m_oldBlAngle.inverse());
	m_oldBlAngle = newBlAngle;

	Translation2D newBlDistance = GetTigerSwerve()->GetModules()->at(BLMODULEID).GetDistance();
	Translation2D deltaBlDistance = newBlDistance.translateBy(m_oldBlDistance.inverse());
	m_oldBlDistance = newBlDistance;

	Rotation2D newBrAngle = GetTigerSwerve()->GetModules()->at(BRMODULEID).GetAngle();
	Rotation2D deltaBrAngle = newBrAngle.rotateBy(m_oldBrAngle.inverse());
	m_oldBrAngle = newBrAngle;

	Translation2D newBrDistance = GetTigerSwerve()->GetModules()->at(BRMODULEID).GetDistance();
	Translation2D deltaBrDistance = newBrDistance.translateBy(m_oldBrDistance.inverse());
	m_oldBrDistance = newBrDistance;

	RigidTransform2D::Delta deltaFlVelocity = RigidTransform2D::Delta::fromDelta(-deltaFlDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaFrVelocity = RigidTransform2D::Delta::fromDelta(-deltaFrDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaBlVelocity = RigidTransform2D::Delta::fromDelta(-deltaBlDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaBrVelocity = RigidTransform2D::Delta::fromDelta(-deltaBrDistance.getX(), 0, 0, deltaTimestamp);

	Rotation2D newGyroYaw = Rotation2D::fromDegrees(-GetTigerDrive()->GetAdjYaw());
	Rotation2D deltaGyroYaw = newGyroYaw.rotateBy(m_oldGyroYaw.inverse());
	m_oldGyroYaw = newGyroYaw;

	observer->UpdateRobotPoseObservation(newFlAngle, deltaFlVelocity,
										 newFrAngle, deltaFrVelocity,
										 newBlAngle, deltaBlVelocity,
										 newBrAngle, deltaBrVelocity, timestamp, deltaGyroYaw);

	RigidTransform2D observerPos = observer->GetLastRobotPose();

	SmartDashboard::PutNumber("Field X", observerPos.getTranslation().getX());
	SmartDashboard::PutNumber("Field Y", observerPos.getTranslation().getY());
	SmartDashboard::PutNumber("Field Yaw", observerPos.getRotation().getDegrees());

	SmartDashboard::PutNumber("FL Rotation Angle", tigerSwerve->GetModules()->at(FLMODULEID).GetAngle().getDegrees());
	SmartDashboard::PutNumber("FR Rotation Angle", tigerSwerve->GetModules()->at(FRMODULEID).GetAngle().getDegrees());
	SmartDashboard::PutNumber("BL Rotation Angle", tigerSwerve->GetModules()->at(BLMODULEID).GetAngle().getDegrees());
	SmartDashboard::PutNumber("BR Rotation Angle", tigerSwerve->GetModules()->at(BRMODULEID).GetAngle().getDegrees());

	SmartDashboard::PutNumber("FL Drive Velocity", tigerSwerve->GetModules()->at(FLMODULEID).GetVelocity());
	SmartDashboard::PutNumber("FR Drive Velocity", tigerSwerve->GetModules()->at(FRMODULEID).GetVelocity());
	SmartDashboard::PutNumber("BL Drive Velocity", tigerSwerve->GetModules()->at(BLMODULEID).GetVelocity());
	SmartDashboard::PutNumber("BR Drive Velocity", tigerSwerve->GetModules()->at(BRMODULEID).GetVelocity());
}
