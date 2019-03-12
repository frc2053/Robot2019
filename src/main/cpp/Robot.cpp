/**
 * \file Robot.cpp
 * \brief The Main Class
 * \author Drew Williams
 */

#include "Robot.h"
#include <frc/commands/Scheduler.h>
#include <Commands/Drive/ZeroYaw.h>
#include "Commands/Drive/ZeroPose.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Commands/Drive/FollowPath.h"
#include "Commands/Drive/FollowPathVelocity.h"
#include "Commands/Drive/DriveCommand.h"
#include <iostream>

std::unique_ptr<OI> Robot::oi;
std::unique_ptr<DrivebaseSubsystem> Robot::drivebaseSubsystem;
std::unique_ptr<IntakeSubsystem> Robot::intakeSubsystem;
std::unique_ptr<LiftSubsystem> Robot::liftSubsystem;
std::unique_ptr<ElevatorSubsystem> Robot::elevatorSubsystem;
std::shared_ptr<ObserverSubsystem> Robot::observer;
std::unique_ptr<PathManager> Robot::pathManager;
std::unique_ptr<RobotMap> Robot::robotMap;
std::unique_ptr<VisionSubsystem> Robot::visionSubsystem;

/**
 * \brief Robot Initialization
 *
 * This is the entry point for the robot program.
 * We have all of the main functions that control how
 * the robot behaves. Usually we init subsystems here and
 * also set up game specific auto routines.
 */
void Robot::RobotInit()
{
	robotMap = std::make_unique<RobotMap>();
	robotMap->Init();

	autoChooser.SetDefaultOption("smoothtenFtForward", "smoothtenFtForward");

	SmartDashboard::PutData("AutoMode", &autoChooser);

	pathManager = std::make_unique<PathManager>();
	observer = std::make_shared<ObserverSubsystem>();
	drivebaseSubsystem = std::make_unique<DrivebaseSubsystem>();
	intakeSubsystem = std::make_unique<IntakeSubsystem>();
	liftSubsystem = std::make_unique<LiftSubsystem>();
	elevatorSubsystem = std::make_unique<ElevatorSubsystem>();
	visionSubsystem = std::make_unique<VisionSubsystem>();
	oi = std::make_unique<OI>();
	SmartDashboard::PutData("Zero Yaw", new ZeroYaw());
	SmartDashboard::PutData("Zero Pose", new ZeroPose());
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture("camera", "/dev/video0");
	camera.SetResolution(320, 240);
	camera.SetFPS(30);
	camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
}

/**
 * \brief Disabled Init
 * 
 * 
 * 
 *
 * This function runs once when you disable the robot.
 * Runs everytime you disable. Might want to use to
 * make sure everything shuts down correctly.
 */
void Robot::DisabledInit() {
	Robot::drivebaseSubsystem->GetTigerDrive()->SetIsRotDoneOverride(true);
	Robot::drivebaseSubsystem->GetTigerDrive()->SetIsRotDone(true);
	Robot::drivebaseSubsystem->MecDrive(0,0,0,Robot::drivebaseSubsystem->GetTigerDrive()->GetAdjYaw());
	Robot::elevatorSubsystem->RunElevatorMotor(0);
}

/**
 * \brief Disabled Periodic
 *
 * This function runs every 20ms while the robot is
 * disabled. Used to check for auto state changes
 * like the scale state in auto.
 *
 */
void Robot::DisabledPeriodic()
{
	frc::Scheduler::GetInstance()->Run();
}

/**
 * \brief Auto Init
 *
 * This function runs once when you first change to
 * autonomous mode. Used to get selected auto mode.
 */
void Robot::AutonomousInit()
{
	//selectedMode = (std::string)autoChooser.GetSelected();
	Command* testAuto = new FollowPath("/home/lvuser/deploy/smoothtenFtForward.csv");

	//ONLY PUT PATH NAME AND .CSV FOR VELOCITY FOLLOW PATH NO HOME/LVUSER AND NO FL FR OR ANYTHING
	//if (selectedMode == "tenFeetForward")
	//{
		//Command *testAuto = new FollowPathVelocity("TenFeetForward.csv");
		testAuto->Start();
	//}
}

/**
 * \brief Auto Periodic
 *
 * This function runs every 20ms while the robot is
 * in autonomous mode. Can be used to check path following
 * or if we are waiting for something in auto.
 */
void Robot::AutonomousPeriodic()
{
	frc::Scheduler::GetInstance()->Run();
	SmartDashboard::PutNumber("PATH GOAL X", drivebaseSubsystem->GetDriveController()->GetFieldTarget().getTranslation().getX());
	SmartDashboard::PutNumber("PATH GOAL Y", drivebaseSubsystem->GetDriveController()->GetFieldTarget().getTranslation().getY());
	SmartDashboard::PutNumber("PATH GOAL YAW", drivebaseSubsystem->GetDriveController()->GetFieldTarget().getRotation().getDegrees());
}

/**
 * \brief Teleop Init
 *
 * This function runs once when you enable the robot in teleop.
 * Runs everytime you enable
 */
void Robot::TeleopInit()
{
	Command *driveCommand = new DriveCommand();
	driveCommand->Start();
}

/**
 * \brief Teleop Periodic
 *
 * This function runs every 20ms while teleop mode is active.
 * Can be used to check for different states like LED status or
 * smartdashboard updates. Make sure you dont overload this function
 * or you will lag the robot!
 */
void Robot::TeleopPeriodic()
{
	frc::Scheduler::GetInstance()->Run();
}

/**
 * \brief Test Periodic
 *
 * This function runs every 20ms while test mode is active.
 * Not used that much...
 */
void Robot::TestPeriodic()
{
}

int main()
{
	return frc::StartRobot<Robot>();
}
