#pragma once

#include <frc/PIDSource.h>
#include "Subsystems/ObserverSubsystem.h"
#include <memory>

class ObserverPIDSourceX: public frc::PIDSource {
public:
	ObserverPIDSourceX(const std::shared_ptr<ObserverSubsystem>& src);
	virtual ~ObserverPIDSourceX();

	double PIDGet();
private:
	std::shared_ptr<ObserverSubsystem> observer;
};
