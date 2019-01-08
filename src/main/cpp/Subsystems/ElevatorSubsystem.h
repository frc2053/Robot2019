/*#ifndef ElevatorSubsystem_H
#define ElevatorSubsystem_H

#include "RobotMap.h"
#include <Commands/Subsystems.h>
#include "WPILib.h"

class ElevatorSubsystem: public frc::Subsystem {
    private:

    std::shared_ptr<can::TalonSRX> primaryMotor;
    std::shared_ptr<can::TalonSRX> followerMotor01;
    std::shared_ptr<can::TalonSRX? followerMotor02;
    std::shard_ptr<frc::DoubleSolenoid> shifterSolenoid;

    public:
    ElevatorSubsystem();
    void InitDefaultCommand();
    void SwitchToElevatorMotor();
    void SwitchToClimbMotor();
    void GoToHeight(double inputHeight);
    int ConvertHeightToTicks(double inputHeight);
    void RunElevatorMotor(double speed);
};

#endif*/
