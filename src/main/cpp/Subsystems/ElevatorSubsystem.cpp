/*#include "ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() : Subsystem("ElevatorSubsystem"){
    std::cout << "Elevator Subsystem Consuctor" << std::endl;
    primaryMotor = RobotMap::elevatorClimberSubsystemPrimaryTalon;
    followerMotor01 = RobotMap::elevatorClimberSubsystemFollower01Talon;
    followerMotor02 = RobotMap::elevatorClimberSubsystemFollower02Talon;
    shifterSolenoid = RobotMap::elevatorClimberSubsystemShifterSolenoid;
}

void ElevatorSubsystem::InitDefaultCommand(){

}

void ElevatorSubsystem::SwitchToElevatorMotor(){
    shifterSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    std::cout << "Changed to elevator" << shifterSolenoid->Get() <<std::endl;
}

void ElevatorSubsystem::SwitchToClimberMotor(){
    shifterSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    std::cout << "Changed to cliber" << shifterSolenoid->Get() << std::endl;
}

void ElevatorSubsystem::GoToHeight(double inputHeight){
    int ticks = ConvertHeightToTicks(inputHeight);
    primaryMotor->Set(ControlMode::Position, ticks);
}

int ElevatorSubsystem::ConvertHeightToTicks(double inputHeight){
    int tickSetpoint = 0;
    tickSetpoint = (inputHeight * 20532);
    return tickSetpoint;
}


void ElevatorSubsystem::RunElevatorMotor(double speed){
    primaryMotor->Set(ControlMode::PercentOutput, speed)
}
}
*/