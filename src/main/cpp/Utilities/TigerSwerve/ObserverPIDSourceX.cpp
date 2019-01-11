#include "Utilities/TigerSwerve/ObserverPIDSourceX.h"

ObserverPIDSourceX::ObserverPIDSourceX(const std::shared_ptr<ObserverSubsystem>& src) {
	observer = src;
}

ObserverPIDSourceX::~ObserverPIDSourceX() {

}

double ObserverPIDSourceX::PIDGet() {
	return observer->GetLastRobotPose().getTranslation().getX();
}
