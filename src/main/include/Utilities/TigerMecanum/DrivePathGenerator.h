/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Utilities/Math/RigidTransform2D.h"
#include <vector>
#include <math.h>
#include <frc/WPILib.h>
#include "Utilities/Math/InterpolatingMap.h"
#include "Utilities/Math/InterpolatingDouble.h"
#include <fstream>
#include <iostream>


struct Waypoint {
  RigidTransform2D pose;
  double maxDistAway;
};

struct TempWaypoint {
  RigidTransform2D pose;
  double maxDistAway;
  int tempPathIndex;
  double distTraveled;
  int finalPathIndex;
};

struct FinalPath {
  RigidTransform2D pose;
  double time;
};

class DrivePathGenerator {
 public:
  DrivePathGenerator();
  virtual ~DrivePathGenerator();
  void GeneratePath(std::vector<Waypoint> &waypoints, double maxSpeed, double maxAccel, double sampleRate);
};
