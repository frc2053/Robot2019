/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <map>
#include <string>
#include "Utilities/TigerMecanum/PathLoader.h"

class PathManager {
public:
  PathManager();
  virtual ~PathManager();
  Path2D& GetPath(std::string);
  bool HasMissingPath();
  void ReloadPaths();
private:
  std::map<std::string, Path2D> m_paths;
  bool m_missingPath;
};
