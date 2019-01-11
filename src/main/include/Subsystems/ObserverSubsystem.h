/**
 * \file ObserverSubsystem.h
 * \brief This class allows us to "observe" the robot state periodically
 * \author FRC 2481
 */

#pragma once

#include "Utilities/Math/RigidTransform2D.h"
#include "Utilities/Math/InterpolatingMap.h"
#include "Utilities/Math/InterpolatingDouble.h"

class ObserverSubsystem {
public:
	ObserverSubsystem();
	virtual ~ObserverSubsystem();

	void ResetPose();
	void ResetPose(RigidTransform2D robotPose);
	void UpdateRobotPoseObservation(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle,
			RigidTransform2D::Delta& frVelocity, Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity,
			Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity, double timeStamp, Rotation2D& deltaGyroYaw);
	RigidTransform2D GetRobotPose(double timestamp);
	void SetRobotPos(RigidTransform2D robotPos, double timestamp);
	RigidTransform2D GetLastRobotPose();
private:
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_robotPos;
	Rotation2D m_oldGyroYaw;
};

