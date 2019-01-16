#pragma once

#include <frc/PIDSource.h>
#include "Subsystems/ObserverSubsystem.h"
#include <memory>

class ObserverPIDSourceYaw: public frc::PIDSource {
public:
	ObserverPIDSourceYaw(const std::shared_ptr<ObserverSubsystem>& src);
	virtual ~ObserverPIDSourceYaw();

	double PIDGet();
private:
	std::shared_ptr<ObserverSubsystem> observer;
};
