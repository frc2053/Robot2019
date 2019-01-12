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

//DRIVEBASE ROTATION PARAMETERS
constexpr double kROTATION_P = .03;
constexpr double kROTATION_I = 0.0;
constexpr double kROTATION_D = .1;
constexpr double kROTATION_ANGLE_TOLERANCE = 2;

//DRIVEBASE TALONS
constexpr int kDRIVESPARK_FL_ID = 1;
constexpr int kDRIVESPARK_FR_ID = 2;
constexpr int kDRIVESPARK_BL_ID = 3;
constexpr int kDRIVESPARK_BR_ID = 4;

//PHYSICAL CONSTANTS
constexpr int kWHEEL_RADIUS = 6;
constexpr double kDRIVE_GEARBOX_RATIO = 55/12;
constexpr int kNEO_TICKS_PER_REV = 42;
constexpr double kTICKS_PER_WHEEL_REV = kNEO_TICKS_PER_REV * kDRIVE_GEARBOX_RATIO * kWHEEL_RADIUS;
constexpr double kWHEEL_BASE_WIDTH = 29;
constexpr double kWHEEL_BASE_LENGTH = 29;