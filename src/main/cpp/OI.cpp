#include "OI.h"
#include "RobotMap.h"

OI::OI() {
    driverJoystick = std::make_unique<TigerJoystick>(JOYSTICK_DRIVER);
}

const std::unique_ptr<TigerJoystick>& OI::GetDriverJoystick() {
    return driverJoystick;
}