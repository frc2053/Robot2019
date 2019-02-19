#pragma once

#include <frc/smartdashboard/SendableBase.h>
#include <frc/smartdashboard/SendableBuilder.h>

class RobotPose : public frc::SendableBase {
private:
    double x;
    double y;
    double heading;
public:
    RobotPose();
    void SetX(double newX);
    void SetY(double newY);
    void SetHeading(double newHeading);
    double GetX();
    double GetY();
    double GetHeading();
    void InitSendable(frc::SendableBuilder& builder) override;
};