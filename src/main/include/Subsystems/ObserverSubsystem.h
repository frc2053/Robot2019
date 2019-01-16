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
	void UpdateRobotPoseObservation(RigidTransform2D::Delta& flVelocity, 
		RigidTransform2D::Delta& frVelocity, RigidTransform2D::Delta& blVelocity, 
		RigidTransform2D::Delta& brVelocity, double timeStamp, Rotation2D& deltaGyroYaw);
	RigidTransform2D GetRobotPose(double timestamp);
	void SetRobotPos(RigidTransform2D robotPos, double timestamp);
	RigidTransform2D GetLastRobotPose();
private:
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_robotPos;
	Rotation2D m_oldGyroYaw;
};

