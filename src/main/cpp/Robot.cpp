#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

std::unique_ptr<DrivebaseSubsystem> Robot::drivebaseSubsystem;
std::unique_ptr<OI> Robot::oi;

void Robot::RobotInit() {
  drivebaseSubsystem = std::make_unique<DrivebaseSubsystem>();
  oi = std::make_unique<OI>();
}

void Robot::RobotPeriodic() {

}

void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() { 
  frc::Scheduler::GetInstance()->Run(); 
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() { 
  frc::Scheduler::GetInstance()->Run(); 
}

//void Robot::TestPeriodic() {

//}

#ifndef RUNNING_FRC_TESTS
int main() { 
  return frc::StartRobot<Robot>(); 
}
#endif