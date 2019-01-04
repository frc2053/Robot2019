#include "ObserverPIDSourceYaw.h"

ObserverPIDSourceYaw::ObserverPIDSourceYaw(const std::shared_ptr<ObserverSubsystem>& src) {
	observer = src;
}

ObserverPIDSourceYaw::~ObserverPIDSourceYaw() {

}

double ObserverPIDSourceYaw::PIDGet() {
	return observer->GetLastRobotPose().getRotation().getDegrees();
}
