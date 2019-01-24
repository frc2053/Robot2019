/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Utilities/TigerMecanum/PathLoader.h"
#include <sstream>

PathLoader::PathLoader(Path2D& path) : m_path(path) {

}

PathLoader::~PathLoader() {

}

void PathLoader::LoadPath(const std::string filePath) {
    std::ifstream fin(filePath);
    double x;
    double y;
    double yaw;
    double time;
    std::string field;

    while(fin.good()) {

        std::getline(fin, field, ',');
        std::istringstream(field) >> time;

        std::getline(fin, field, ',');
        std::istringstream(field) >> x;

        std::getline(fin, field, ',');
        std::istringstream(field) >> y;

        std::getline(fin, field, ',');
        std::istringstream(field) >> yaw;

        m_path.put(InterpolatingDouble(time), RigidTransform2D(Translation2D(x, y), Rotation2D::fromDegrees(yaw)));
    }
}
