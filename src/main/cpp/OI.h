#pragma once

#include "Utilities/TigerJoystick/TigerJoystick.h"

class OI {
    public:
    OI();
    const std::unique_ptr<TigerJoystick>& GetDriverJoystick();
    private:
    std::unique_ptr<TigerJoystick> driverJoystick;
};