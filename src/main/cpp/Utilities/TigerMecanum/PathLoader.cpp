/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Utilities/TigerMecanum/PathLoader.h"
#include <sstream>
#include <iostream>
#include <cstring>
PathLoader::PathLoader(Path2D &path) : m_path(path)
{
}

PathLoader::~PathLoader()
{
}

void PathLoader::LoadPath(const std::string filePath)
{
    std::ifstream fin(filePath);
    std::string line;
    std::getline(fin, line);

    std::stringstream lineStream(line);
    std::string cell;

    double x = 0;
    double y = 0;
    double yaw = 0;
    double time = 0;

    while(fin.good()) {
        
    }

    // m_path.put(InterpolatingDouble(0), RigidTransform2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.02), RigidTransform2D(Translation2D(0, 0.078939), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.04), RigidTransform2D(Translation2D(0, 0.26423), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.06), RigidTransform2D(Translation2D(0, 0.593635), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.08), RigidTransform2D(Translation2D(0, 1.10833), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.10), RigidTransform2D(Translation2D(0, 1.828904), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.12), RigidTransform2D(Translation2D(0, 2.755356), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.14), RigidTransform2D(Translation2D(0, 3.887686), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.16), RigidTransform2D(Translation2D(0, 5.225895), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.18), RigidTransform2D(Translation2D(0, 6.749394), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.20), RigidTransform2D(Translation2D(0, 8.417007), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.22), RigidTransform2D(Translation2D(0, 10.18756), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.24), RigidTransform2D(Translation2D(0, 12.019168), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.26), RigidTransform2D(Translation2D(0, 13.849359), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.28), RigidTransform2D(Translation2D(0, 15.61637), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.30), RigidTransform2D(Translation2D(0, 17.279025), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.32), RigidTransform2D(Translation2D(0, 18.796149), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.34), RigidTransform2D(Translation2D(0, 20.127273), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.36), RigidTransform2D(Translation2D(0, 21.25252), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.38), RigidTransform2D(Translation2D(0, 22.171889), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.40), RigidTransform2D(Translation2D(0, 22.885379), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.42), RigidTransform2D(Translation2D(0, 23.393699), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.44), RigidTransform2D(Translation2D(0, 23.718146), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.46), RigidTransform2D(Translation2D(0, 23.899894), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.48), RigidTransform2D(Translation2D(0, 23.980121), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.50), RigidTransform2D(Translation2D(0, 24), Rotation2D::fromDegrees(0)));
    // m_path.put(InterpolatingDouble(0.52), RigidTransform2D(Translation2D(0, 24), Rotation2D::fromDegrees(0)));
}
