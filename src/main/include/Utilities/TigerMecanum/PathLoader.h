/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <fstream>
#include "Utilities/Math/RigidTransform2D.h"
#include "Utilities/Math/InterpolatingMap.h"
#include "Utilities/Math/InterpolatingDouble.h"

typedef InterpolatingMap<InterpolatingDouble, RigidTransform2D> Path2D;

class PathLoader {
public:
  PathLoader(Path2D& path);
  virtual ~PathLoader();
  void LoadPath(const std::string filePath);
private:
  Path2D& m_path;
};
