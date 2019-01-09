#indef DrivebaseSubsystem_H
#define DrivebaseSubsystem_H

#include "../RobotMap.h"
#include <Commands/Subsystem.h>

class DrivebaseSubsystem : public frc::Subsystem{
    public:
    DrivebaseSubsystem();
    void InitDefaultCommand();
    void TankDrive();
    void ConfigureTalons();
    void Periodic();

    std::shard_ptr<TigerDrive> tigerDrive;
    private:
    std::shared_ptr<can::TalonSRX> leftLeaderTalon;
    std::shared_ptr<can::TalonSrx> leftFollowerTalon;
    std::shared_ptr<can::TalonSRX> rightLeaderTalon;
    std::shared_ptr<can::TalonSRX> rightFollowerTalon;
    //cal stuff?
};

#endif