/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Utilities/TigerMecanum/PathLoader.h"
#include <string>
#include <fstream>
#include <sstream>
#include "Robot.h"
#include <iostream>

PathLoader::PathLoader(Path2D &path) : m_path(path)
{
}

PathLoader::PathLoader() {
}

PathLoader::~PathLoader()
{
}

void PathLoader::LoadPath(const std::string filePath)
{
    std::ifstream testFile(filePath);
    while (testFile.good()) {
		std::vector<std::string> results = GetNextLineAndSplitIntoTokens(testFile);
		if(results.size() == 4){
			m_path.put(InterpolatingDouble(std::stod(results[0])), RigidTransform2D(Translation2D(std::stod(results[1]) * Robot::robotMap->kSTRAFE_MULTIPLIER, std::stod(results[2])), Rotation2D::fromDegrees(std::stod(results[3]))));
			std::cout << "Time: " << results[0] << " X: " << results[1] << " Y: " << results[2] << " Heading: " << results[3] << "\n";
		}
	}
}

std::vector<double> PathLoader::LoadVelocityPath(const std::string filePath) {
	std::ifstream testFile(filePath);
	std::vector<double> retVal;
    while (testFile.good()) {
		std::vector<std::string> results = GetNextLineAndSplitIntoTokens(testFile);
		if(results.size() == 2){
			retVal.push_back(std::stod(results[1]));
		}
	}
	return retVal;
}

std::vector<std::string> PathLoader::GetNextLineAndSplitIntoTokens(std::istream& str)
{
	std::vector<std::string> result;
	std::string line;
	std::getline(str, line);

	std::stringstream lineStream(line);
	std::string cell;

	while (std::getline(lineStream, cell, ','))
	{
		result.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!lineStream && cell.empty())
	{
		// If there was a trailing comma then add an empty element.
		result.push_back("");
	}
	return result;
}