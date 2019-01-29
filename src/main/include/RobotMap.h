/**
 * \file RobotMap.h
 * \brief Holds constants for the robot
 * \author Drew Williams
 *
 * Here we hold all of the robots constants.
 * These are the default values when the robot starts up.
 */
#pragma once

#include <string>
#include <iostream>
#include <sstream>

class RobotMap {
public: 
    RobotMap();
    void Init();

    //CONTROLLER STUFF
    int kDRIVER_CONTROLLER_PORT;
    int kOPERATOR_CONTROLLER_PORT;
    double  kCONTROLLER_PERIOD;

    //ENCODER STUFF
    int kTICKS_PER_REV_OF_ENCODER;
    int kTICKS_PER_REV_NEO;
    int kELEVATORTICKS_PER_INCH;
    double  kWRIST_GEAR_RATIO;
    double  kFLAPPER_GEAR_RATIO;
    double  kWHEEL_DIAMETER;
    double  kDRIVE_GEAR_RATIO;
    double  kWHEEL_BASE_LENGTH;
    double  kWHEEL_BASE_WIDTH;

    //DRIVEBASE ROTATION PARAMETERS
    double  kROTATION_P;
    double  kROTATION_I;
    double  kROTATION_D;
    double  kROTATION_ANGLE_TOLERANCE;

    //ELEVATOR PID
    double  kELEVATOR_F;
    double  kELEVATOR_P;
    double  kELEVATOR_I;
    double  kELEVATOR_D;

    //DRIVEBASE TALONS
    int kDRIVESPARK_FL_ID;
    int kDRIVESPARK_FR_ID;
    int kDRIVESPARK_BL_ID;
    int kDRIVESPARK_BR_ID;

    //INTAKE TALONS
    int kINTAKE_WHEELS_ID;
    int kINTAKE_ACTUATOR_ID;

    //ELEVATOR TALONS
    int kELEVATOR_LEADER_ID;
    int kELEVATOR_FOLLOWERONE_ID;
    int kELEVATOR_FOLLOWERTWO_ID;

    //FLAPPER TALONS
    int kINTAKE_FLAPPER_LEFT_ID;
    int kINTAKE_FLAPPER_RIGHT_ID;

    //INTAKE PARAMS
    double  kINTAKE_ANGLE_BALL;
    double  kINTAKE_ANGLE_UP;
    double  kFLAPPER_UP_ANGLE;
    //;)
    double  kFLAPPER_DOWN_ANGLE;
    int kINTAKE_ANGLE_TOLERANCE;
    double  kINTAKE_SPEED;

    //ELEVATOR PARAMS
    double  kELEVATOR_GROUND;
    double  kELEVATOR_LEVEL_ONE_HATCH;
    double  kELEVATOR_LEVEL_TWO_HATCH;
    double  kELEVATOR_LEVEL_THREE_HATCH;
    double  kELEVATOR_LEVEL_ONE_PORT;
    double  kELEVATOR_LEVEL_TWO_PORT;
    double  kELEVATOR_LEVEL_THREE_PORT;

    //FORWARD KINEMATICS
    double  kFORWARD_KINEMATICS_WEIGHT;
    double  kGYRO_WEIGHT;
    double  kAUTO_CONTROLLER_P;
    double  kAUTO_CONTROLLER_I;
    double  kAUTO_CONTROLLER_D;
    double  kAUTO_CONTROLLER_V;

    double  kENCODER_REVS_PER_WHEEL_REV;

    double  kTOLERANCE_POS;
    double  kTOLERANCE_HEADING;

    double  kSTRAFE_MULTIPLIER;

    //LIFT STUFF
    int kFOOT_SPARK_ID;
    int kLEG_LEADER_TALON_ID;
    int kLEG_FOLLOWER_TALON_ID;
};