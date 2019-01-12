/**
 * \file RobotMap.h
 * \brief Holds constants for the robot
 * \author Drew Williams
 *
 * Here we hold all of the robots constants.
 * These are the default values when the robot starts up.
 */

#include <string>

//CONTROLLER STUFF
constexpr int kDRIVER_CONTROLLER_PORT = 0;
constexpr int kOPERATOR_CONTROLLER_PORT = 1;

//ENCODER STUFF
constexpr int kTICKS_PER_REV_OF_ENCODER = 4096;

//DRIVEBASE ROTATION PARAMETERS
constexpr double kROTATION_P = .03;
constexpr double kROTATION_I = 0.0;
constexpr double kROTATION_D = .1;
constexpr double kROTATION_ANGLE_TOLERANCE = 2;

//DRIVEBASE TALONS
constexpr int kDRIVESPARK_FL_ID = 4;
constexpr int kDRIVESPARK_FR_ID = 1;
constexpr int kDRIVESPARK_BL_ID = 3;
constexpr int kDRIVESPARK_BR_ID = 2;

//INTAKE TALONS
constexpr int kINTAKE_WHEELS_ID = 5;
constexpr int kINTAKE_ACTUATOR_ID = 6;

//INTAKE PARAMS
constexpr double kINTAKE_ANGLE_BALL = 45;
constexpr double kINTAKE_ANGLE_UP = 0;
constexpr double kINTAKE_ANGLE_HATCH = 90;
constexpr int kINTAKE_ANGLE_TOLERANCE = 5;
constexpr double kINTAKE_SPEED = 1;