/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Utilities/TigerMecanum/PathManager.h"

PathManager::PathManager() {
    m_missingPath = false;
}

PathManager::~PathManager() {

}

Path2D& PathManager::GetPath(std::string pathName) {
    auto result = m_paths.find(pathName);
    if(result == m_paths.end()) {
        Path2D& path = m_paths[pathName];
        PathLoader pathLoader(path);
        pathLoader.LoadPath(pathName);

        if(path.empty()) {
            m_missingPath = true;
            printf("Missing path %s\n", pathName.c_str());
        }
    }
    return m_paths[pathName];
}

bool PathManager::HasMissingPath() {
    return m_missingPath;
}

void PathManager::ReloadPaths() {
    m_missingPath = false;
    for(auto pathPair : m_paths) {
        Path2D& path = pathPair.second;
        path.clear();
        PathLoader pathLoader(path);
        pathLoader.LoadPath(pathPair.first);

        if(path.empty() && !pathPair.first.empty()) {
            m_missingPath = true;
            printf("Missing path %s\n", pathPair.first.c_str());
        }
    }
}