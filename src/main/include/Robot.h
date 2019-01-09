#pragma once

#include <frc/TimedRobot.h>
#include "OI.h"
#include "Subsystems/DrivebaseSubsystem.h"

class Robot : public frc::TimedRobot{
    public: 
    static std::unique_ptr<DrivebaseSubsystem> drivebaseSubsystem;
    static std::unique_ptr<OI> oi;

    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    private:
    //nothing ig
};