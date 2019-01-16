#pragma once

#include "Subsystems/ObserverSubsystem.h"
#include <memory>
#include "ObserverPIDSourceX.h"
#include "ObserverPIDSourceY.h"
#include "ObserverPIDSourceYaw.h"
#include "Utilities/Math/PIDController2481.h"
#include "DriveControllerOutput.h"
#include "Utilities/Math/RigidTransform2D.h"

class DriveController {
public:
	DriveController(const std::shared_ptr<ObserverSubsystem>& observerPtr);
	virtual ~DriveController();

	void SetFieldTarget(RigidTransform2D fieldTarget);
	void SetRobotTarget(RigidTransform2D robotTarget);
	void EnableController();
	bool IsOnTarget();

	RigidTransform2D GetControllerError();
	RigidTransform2D GetDriveControlSignal();
private:
	std::unique_ptr<frc::PIDController2481> positionXController;
	std::unique_ptr<frc::PIDController2481> positionYController;
	std::unique_ptr<frc::PIDController2481> positionYawController;

	std::unique_ptr<ObserverPIDSourceX> positionXSource;
	std::unique_ptr<ObserverPIDSourceY> positionYSource;
	std::unique_ptr<ObserverPIDSourceYaw> positionYawSource;

	std::unique_ptr<DriveControllerOutput> positionXSignal;
	std::unique_ptr<DriveControllerOutput> positionYSignal;
	std::unique_ptr<DriveControllerOutput> positionYawSignal;

	std::shared_ptr<ObserverSubsystem> observer;

	void ResetController();
};
