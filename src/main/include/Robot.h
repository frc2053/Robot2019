/**
 * \file Robot.h
 * \brief The Main Class
 * \author Drew Williams
 */

#pragma once

#include <frc/TimedRobot.h>
#include "OI.h"
#include "Subsystems/DrivebaseSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/ObserverSubsystem.h"

/**
 * This is the main class of the robot. Sort of like
 * a main.cpp file.
 */
class Robot : public frc::TimedRobot {
public:
	static std::unique_ptr<OI> oi;
	static std::unique_ptr<DrivebaseSubsystem> drivebaseSubsystem;
	static std::unique_ptr<ObserverSubsystem> observer;
	static std::unique_ptr<IntakeSubsystem> intakeSubsystem;

	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
private:
};
