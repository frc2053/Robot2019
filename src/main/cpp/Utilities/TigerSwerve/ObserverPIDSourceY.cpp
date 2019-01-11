#include "Utilities/TigerSwerve/ObserverPIDSourceY.h"

ObserverPIDSourceY::ObserverPIDSourceY(const std::shared_ptr<ObserverSubsystem>& src) {
	observer = src;
}

ObserverPIDSourceY::~ObserverPIDSourceY() {

}

double ObserverPIDSourceY::PIDGet() {
	return observer->GetLastRobotPose().getTranslation().getY();
}
