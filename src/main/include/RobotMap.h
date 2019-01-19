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
constexpr double kCONTROLLER_PERIOD = 0.02;

//ENCODER STUFF
constexpr int kTICKS_PER_REV_OF_ENCODER = 4096;
constexpr int kTICKS_PER_REV_NEO = 48;
constexpr int kELEVATORTICKS_PER_INCH= 4096;
constexpr double kWRIST_GEAR_RATIO = 1.8181;
constexpr double kFLAPPER_GEAR_RATIO = 1;
constexpr double kWHEEL_DIAMETER = 6;
constexpr double kDRIVE_GEAR_RATIO = 7.86;
constexpr double kWHEEL_BASE_LENGTH = 25.25;
constexpr double kWHEEL_BASE_WIDTH = 25.25;

//DRIVEBASE ROTATION PARAMETERS
constexpr double kROTATION_P = .03;
constexpr double kROTATION_I = 0.0;
constexpr double kROTATION_D = .1;
constexpr double kROTATION_ANGLE_TOLERANCE = 2;

//ELEVATOR PID
constexpr double kELEVATOR_F = 0;
constexpr double kELEVATOR_P = 1;
constexpr double kELEVATOR_I = 0;
constexpr double kELEVATOR_D = 0;

//DRIVEBASE TALONS
constexpr int kDRIVESPARK_FL_ID = 4;
constexpr int kDRIVESPARK_FR_ID = 1;
constexpr int kDRIVESPARK_BL_ID = 3;
constexpr int kDRIVESPARK_BR_ID = 2;

//INTAKE TALONS
constexpr int kINTAKE_WHEELS_ID = 5;
constexpr int kINTAKE_ACTUATOR_ID = 6;

//ELEVATOR TALONS
constexpr int kELEVATOR_LEADER_ID = 7;
constexpr int kELEVATOR_FOLLOWERONE_ID = 8;
constexpr int kELEVATOR_FOLLOWERTWO_ID = 9;

//FLAPPER TALONS
constexpr int kINTAKE_FLAPPER_LEFT_ID = 10;
constexpr int kINTAKE_FLAPPER_RIGHT_ID = 11;

//INTAKE PARAMS
constexpr double kINTAKE_ANGLE_BALL = 45;
constexpr double kINTAKE_ANGLE_UP = 0;
constexpr double kINTAKE_ANGLE_HATCH = 90;
constexpr int kINTAKE_ANGLE_TOLERANCE = 5;
constexpr double kINTAKE_SPEED = 1;

//FORWARD KINEMATICS
constexpr double kFORWARD_KINEMATICS_WEIGHT = 0;
constexpr double kGYRO_WEIGHT = 1;
constexpr double kAUTO_CONTROLLER_P = 1;
constexpr double kAUTO_CONTROLLER_I = 0;
constexpr double kAUTO_CONTROLLER_D = 0;
constexpr double kAUTO_CONTROLLER_V = 0;

constexpr double kENCODER_REVS_PER_WHEEL_REV = .17;

//LIFT STUFF
constexpr int kFOOT_TALON_ID = 12;
constexpr int kLEG_LEADER_TALON_ID = 13;
constexpr int kLEG_FOLLOWER_TALON_ID = 14;