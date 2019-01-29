/**
 * \file RobotMap.h
 * \brief Holds constants for the robot
 * \author Drew Williams
 *
 * Here we hold all of the robots constants.
 * These are the default values when the robot starts up.
 */

#include <string>
#include <iostream>
#include <sstream>
#include "INIReader.h"

class RobotMap {
public: 

//CONTROLLER STUFF
static int kDRIVER_CONTROLLER_PORT;
static int kOPERATOR_CONTROLLER_PORT;
static double  kCONTROLLER_PERIOD;

//ENCODER STUFF
static int kTICKS_PER_REV_OF_ENCODER;
static int kTICKS_PER_REV_NEO;
static int kELEVATORTICKS_PER_INCH;
static double  kWRIST_GEAR_RATIO;
static double  kFLAPPER_GEAR_RATIO;
static double  kWHEEL_DIAMETER;
static double  kDRIVE_GEAR_RATIO;
static double  kWHEEL_BASE_LENGTH;
static double  kWHEEL_BASE_WIDTH;

//DRIVEBASE ROTATION PARAMETERS
static double  kROTATION_P;
static double  kROTATION_I;
static double  kROTATION_D;
static double  kROTATION_ANGLE_TOLERANCE;

//ELEVATOR PID
static double  kELEVATOR_F;
static double  kELEVATOR_P;
static double  kELEVATOR_I;
static double  kELEVATOR_D;

//DRIVEBASE TALONS
static int kDRIVESPARK_FL_ID;
static int kDRIVESPARK_FR_ID;
static int kDRIVESPARK_BL_ID;
static int kDRIVESPARK_BR_ID;

//INTAKE TALONS
static int kINTAKE_WHEELS_ID;
static int kINTAKE_ACTUATOR_ID;

//ELEVATOR TALONS
static int kELEVATOR_LEADER_ID;
static int kELEVATOR_FOLLOWERONE_ID;
static int kELEVATOR_FOLLOWERTWO_ID;

//FLAPPER TALONS
static int kINTAKE_FLAPPER_LEFT_ID;
static int kINTAKE_FLAPPER_RIGHT_ID;

//INTAKE PARAMS
static double  kINTAKE_ANGLE_BALL;
static double  kINTAKE_ANGLE_UP;
static double  kFLAPPER_UP_ANGLE;
//;)
static double  kFLAPPER_DOWN_ANGLE;
static int kINTAKE_ANGLE_TOLERANCE;
static double  kINTAKE_SPEED;

//ELEVATOR PARAMS
static double  kELEVATOR_GROUND;
static double  kELEVATOR_LEVEL_ONE_HATCH;
static double  kELEVATOR_LEVEL_TWO_HATCH;
static double  kELEVATOR_LEVEL_THREE_HATCH;
static double  kELEVATOR_LEVEL_ONE_PORT;
static double  kELEVATOR_LEVEL_TWO_PORT;
static double  kELEVATOR_LEVEL_THREE_PORT;

//FORWARD KINEMATICS
static double  kFORWARD_KINEMATICS_WEIGHT;
static double  kGYRO_WEIGHT;
static double  kAUTO_CONTROLLER_P;
static double  kAUTO_CONTROLLER_I;
static double  kAUTO_CONTROLLER_D;
static double  kAUTO_CONTROLLER_V;

static double  kENCODER_REVS_PER_WHEEL_REV;

static double  kTOLERANCE_POS;
static double  kTOLERANCE_HEADING;

static double  kSTRAFE_MULTIPLIER;

//LIFT STUFF
static int kFOOT_SPARK_ID;
static int kLEG_LEADER_TALON_ID;
static int kLEG_FOLLOWER_TALON_ID;
};

extern RobotMap globalRobotMap;