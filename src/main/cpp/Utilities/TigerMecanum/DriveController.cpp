#include "Utilities/TigerMecanum/DriveController.h"
#include "Robot.h"
#include "Utilities/Math/Rotation2D.h"
#include <iostream>

DriveController::DriveController(const std::shared_ptr<ObserverSubsystem>& observerPtr) {
	observer = observerPtr;

	positionXSource = std::make_unique<ObserverPIDSourceX>(observerPtr);
	positionYSource = std::make_unique<ObserverPIDSourceY>(observerPtr);
	positionYawSource = std::make_unique<ObserverPIDSourceYaw>(observerPtr);

	positionXSignal = std::make_unique<DriveControllerOutput>();
	positionYSignal = std::make_unique<DriveControllerOutput>();
	positionYawSignal = std::make_unique<DriveControllerOutput>();

	positionXController = std::make_unique<frc::PIDController2481>(Robot::robotMap->kAUTO_CONTROLLER_P, Robot::robotMap->kAUTO_CONTROLLER_I, Robot::robotMap->kAUTO_CONTROLLER_D, Robot::robotMap->kAUTO_CONTROLLER_V, positionXSource.get(), positionXSignal.get(), Robot::robotMap->kCONTROLLER_PERIOD);
	positionYController = std::make_unique<frc::PIDController2481>(Robot::robotMap->kAUTO_CONTROLLER_P, Robot::robotMap->kAUTO_CONTROLLER_I, Robot::robotMap->kAUTO_CONTROLLER_D, Robot::robotMap->kAUTO_CONTROLLER_V, positionYSource.get(), positionYSignal.get(), Robot::robotMap->kCONTROLLER_PERIOD);
	positionYawController = std::make_unique<frc::PIDController2481>(Robot::robotMap->kROTATION_P, Robot::robotMap->kROTATION_I, Robot::robotMap->kROTATION_D, 0, positionYawSource.get(), positionYawSignal.get(), Robot::robotMap->kCONTROLLER_PERIOD);

	positionYawController->SetInputRange(-180, 180);

	positionXController->SetOutputRange(-1, 1);
	positionYController->SetOutputRange(-1, 1);

	positionYawController->SetContinuous(true);
	positionYawController->SetInputRange(-180.0, 180.0);
	positionYawController->SetOutputRange(-1.0, 1.0);
	positionYawController->SetAbsoluteTolerance(Robot::robotMap->kROTATION_ANGLE_TOLERANCE);


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

RigidTransform2D DriveController::GetFieldTarget() {
	return RigidTransform2D(Translation2D(positionXController->GetSetpoint(), positionYController->GetSetpoint()), Rotation2D::fromDegrees(positionYawController->GetSetpoint()));
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
	std::cout << "positionYawSignal: " << positionYawSignal->GetOutput() << "\n";
	//;)
	std::cout << "positionYawSource: " << positionYawSource->PIDGet() << "\n";
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
