#pragma once

#include <PIDSource.h>
#include "../../Subsystems/ObserverSubsystem.h"
#include <memory>

class ObserverPIDSourceY: public frc::PIDSource {
public:
	ObserverPIDSourceY(const std::shared_ptr<ObserverSubsystem>& src);
	virtual ~ObserverPIDSourceY();

	double PIDGet();
private:
	std::shared_ptr<ObserverSubsystem> observer;
};
