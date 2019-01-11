#include "Utilities/TigerSwerve/DriveController.h"
#include "RobotMap.h"
#include "Utilities/Math/Rotation2D.h"
DriveController::DriveController(const std::shared_ptr<ObserverSubsystem>& observerPtr) {
	observer = observerPtr;

	positionXSource = std::make_unique<ObserverPIDSourceX>(observerPtr);
	positionYSource = std::make_unique<ObserverPIDSourceY>(observerPtr);
	positionYawSource = std::make_unique<ObserverPIDSourceYaw>(observerPtr);

	positionXSignal = std::make_unique<DriveControllerOutput>();
	positionYSignal = std::make_unique<DriveControllerOutput>();
	positionYawSignal = std::make_unique<DriveControllerOutput>();

	positionXController = std::make_unique<frc::PIDController2481>(kAUTO_P, kAUTO_I, kAUTO_D, kAUTO_V, positionXSource.get(), positionXSignal.get(), kCONTROLLER_PERIOD);
	positionYController = std::make_unique<frc::PIDController2481>(kAUTO_P, kAUTO_I, kAUTO_D, kAUTO_V, positionYSource.get(), positionYSignal.get(), kCONTROLLER_PERIOD);
	positionYawController = std::make_unique<frc::PIDController2481>(kROTATION_P, kROTATION_I, kROTATION_D, kAUTO_V, positionYawSource.get(), positionYawSignal.get(), kCONTROLLER_PERIOD);

	positionYawController->SetInputRange(-180, 180);
	positionYawController->SetContinuous(true);

	positionXController->SetIZone(0);
	positionYController->SetIZone(0);
	positionYawController->SetIZone(0);
}

DriveController::~DriveController() {

}

void DriveController::SetFieldTarget(RigidTransform2D fieldTarget) {
	positionXController->SetSetpoint(fieldTarget.getTranslation().getX());
	positionYController->SetSetpoint(fieldTarget.getTranslation().getY());
	positionYawController->SetSetpoint(fieldTarget.getRotation().getDegrees());
}

void DriveController::SetRobotTarget(RigidTransform2D robotTarget) {
	positionXController->SetSetpoint(robotTarget.getTranslation().getX());
	positionYController->SetSetpoint(robotTarget.getTranslation().getY());
	positionYawController->SetSetpoint(robotTarget.getRotation().getDegrees());
}

bool DriveController::IsOnTarget() {
	bool onTarget = positionXController->OnTarget() && positionYController->OnTarget() && positionXController->OnTarget();
	return onTarget;
}

RigidTransform2D DriveController::GetDriveControlSignal() {
	Translation2D controlSignalTranslation;
	Rotation2D controlSignalRotation;

	controlSignalTranslation.setX(positionXSignal->GetOutput());
	controlSignalTranslation.setY(positionYSignal->GetOutput());
	controlSignalRotation = Rotation2D::fromDegrees(positionYawSignal->GetOutput());

	Rotation2D robotYaw = observer->GetLastRobotPose().getRotation();
	RigidTransform2D driveControlSignal(controlSignalTranslation.rotateBy(robotYaw.inverse()), controlSignalRotation);

	return driveControlSignal;
}


RigidTransform2D DriveController::GetControllerError() {
	RigidTransform2D error;

	error.getTranslation().setX(positionXController->GetError());
	error.getTranslation().setY(positionYController->GetError());
	error.setRotation(Rotation2D::fromDegrees(positionYawController->GetError()));

	return error;
}

void DriveController::ResetController() {
	positionXController->Reset();
	positionYController->Reset();
	positionYawController->Reset();
}

void DriveController::EnableController() {
	ResetController();
	positionXController->Enable();
	positionYController->Enable();
	positionYawController->Enable();
}
