#include "RobotMap.h"
#include "INIReader.h"
#include <iostream>

RobotMap::RobotMap(){

}

void RobotMap::Init() {
    INIReader reader("/home/lvuser/deploy/robotprops.ini");

    if(reader.ParseError() < 0)
    {
        std::cout << "nope";
    }
    
    //CONTROLLER STUFF
    kDRIVER_CONTROLLER_PORT = reader.GetInteger("controller", "kDRIVER_CONTROLLER_PORT", -1);
    kOPERATOR_CONTROLLER_PORT = reader.GetInteger("controller", "kOPERATOR_CONTROLLER_PORT", -1);
    kCONTROLLER_PERIOD = reader.GetReal("controller", "kCONTROLLER_PERIOD", -1.0);

    //ENCODER STUFF
    kTICKS_PER_REV_OF_ENCODER = reader.GetInteger("encoder", "kTICKS_PER_REV_OF_ENCODER", -1);
    kTICKS_PER_REV_NEO = reader.GetInteger("encoder", "kTICKS_PER_REV_NEO", -1);
    kELEVATORTICKS_PER_INCH = reader.GetInteger("encoder", "kELEVATORTICKS_PER_INCH", -1);
    kWRIST_GEAR_RATIO = reader.GetReal("encoder", "kWRIST_GEAR_RATIO", -1.0);
    kFLAPPER_GEAR_RATIO = reader.GetReal("encoder", "kFLAPPER_GEAR_RATIO", -1.0);
    kWHEEL_DIAMETER = reader.GetReal("encoder", "kWHEEL_DIAMETER", -1.0);
    kDRIVE_GEAR_RATIO = reader.GetReal("encoder", "kDRIVE_GEAR_RATIO", -1.0);
    kWHEEL_BASE_LENGTH = reader.GetReal("encoder", "kWHEEL_BASE_LENGTH", -1.0);
    kWHEEL_BASE_WIDTH = reader.GetReal("encoder", "kWHEEL_BASE_WIDTH", -1);

    //DRIVEBASE ROTATION PARAMETERS
    kROTATION_P = reader.GetReal("drivebase rotation", "kROTATION_P", -1.0);
    kROTATION_I = reader.GetReal("drivebase rotation", "kROTATION_I", -1.0);
    kROTATION_D = reader.GetReal("drivebase rotation", "kROTATION_D", -1.0);
    kROTATION_ANGLE_TOLERANCE = reader.GetReal("drivebase rotation", "kROTATION_ANGLE_TOLERANCE", -1.0);

    //ELEVATOR PID
    kELEVATOR_F = reader.GetReal("elevator pid", "kROTATION_F", -1.0);
    kELEVATOR_P = reader.GetReal("elevator pid", "kROTATION_P", -1.0);
    kELEVATOR_I = reader.GetReal("elevator pid", "kROTATION_I", -1.0);
    kELEVATOR_D = reader.GetReal("elevator pid", "kROTATION_D", -1.0);

    //DRIVEBASE TALONS
    kDRIVESPARK_FL_ID = reader.GetReal("drivebase talons", "kDRIVE_SPARK_FL_ID", -1);
    kDRIVESPARK_FR_ID = reader.GetReal("drivebase talons", "kDRIVE_SPARK_FR_ID", -1);
    kDRIVESPARK_BL_ID = reader.GetReal("drivebase talons", "kDRIVE_SPARK_BL_ID", -1);
    kDRIVESPARK_BR_ID = reader.GetReal("drivebase talons", "kDRIVE_SPARK_BR_ID", -1);

    //INTAKE TALONS
    kINTAKE_WHEELS_ID = reader.GetInteger("intake talons", "kDRIVE_SPARK_FL_ID", -1);
    kINTAKE_ACTUATOR_ID = reader.GetInteger("intake talons", "kDRIVE_SPARK_ACTUATOR_ID", -1);

    //ELEVATOR TALONS
    kELEVATOR_LEADER_ID = reader.GetInteger("elevator talons", "kELEVATOR_LEADER_ID", -1);
    kELEVATOR_FOLLOWERONE_ID = reader.GetInteger("elevator talons", "kELEVATOR_FOLLOWER_ONE_ID", -1);
    kELEVATOR_FOLLOWERTWO_ID = reader.GetInteger("elevator talons", "kELEVATOR_FOLLOWER_TWO_ID", -1);

    //FLAPPER TALONS
    kINTAKE_FLAPPER_LEFT_ID = reader.GetInteger("flapper talons", "kINTAKE_FLAPPER_LEFT_ID", -1);
    kINTAKE_FLAPPER_RIGHT_ID = reader.GetInteger("flapper talons", "kINTAKE_FLAPPER_RIGHT_ID", -1);

    //INTAKE PARAMS
    kINTAKE_ANGLE_BALL = reader.GetReal("intake params", "kINTAKE_ANGLE_BALL", -1.0);
    kINTAKE_ANGLE_UP = reader.GetReal("intake params", "kINTAKE_ANGLE_UP", -1.0);
    kFLAPPER_UP_ANGLE = reader.GetReal("intake params", "kFLAPPER_UP_ANGLE", -1.0);
    kFLAPPER_DOWN_ANGLE = reader.GetReal("intake params", "kFLAPPER_DOWN_ANGLE", -1.0);
    //;)
    kINTAKE_ANGLE_TOLERANCE = reader.GetInteger("intake params", "kINTAKE_ANGLE_TOLERANCE", -1);
    kINTAKE_SPEED = reader.GetReal("intake params", "kINTAKE_SPEED", -1.0);

    //ELEVATOR PARAMS
    kELEVATOR_GROUND = reader.GetReal("elevator params", "kELEVATOR_GROUND", -1.0);
    kELEVATOR_LEVEL_ONE_HATCH = reader.GetReal("elevator params", "kELEVATOR_LEVEL_ONE_HATCH", -1.0);
    kELEVATOR_LEVEL_TWO_HATCH = reader.GetReal("elevator params", "kELEVATOR_LEVEL_TWO_HATCH", -1.0);
    kELEVATOR_LEVEL_THREE_HATCH = reader.GetReal("elevator params", "kELEVATOR_LEVEL_THREE_HATCH", -1.0);
    kELEVATOR_LEVEL_ONE_PORT = reader.GetReal("elevator params", "kELEVATOR_LEVEL_ONE_PORT", -1.0);
    kELEVATOR_LEVEL_TWO_PORT = reader.GetReal("elevator params", "kELEVATOR_LEVEL_TWO_PORT", -1.0);
    kELEVATOR_LEVEL_THREE_PORT = reader.GetReal("elevator params", "kELEVATOR_LEVEL_THREE_PORT", -1.0);

    //FORWARD KINEMATICS
    kFORWARD_KINEMATICS_WEIGHT = reader.GetReal("forward kinematics", "kFORWARD_KINEMATICS_WEIGHT", -1.0);
    kGYRO_WEIGHT = reader.GetReal("forward kinematics", "kGYRO_WEIGHT", -1.0);
    kAUTO_CONTROLLER_P = reader.GetReal("forward kinematics", "kAUTO_CONTROLLER_P", -1.0);
    kAUTO_CONTROLLER_I = reader.GetReal("forward kinematics", "kAUTO_CONTROLLER_I", -1.0);
    kAUTO_CONTROLLER_D = reader.GetReal("forward kinematics", "kAUTO_CONTROLLER_D", -1.0);
    kAUTO_CONTROLLER_V = reader.GetReal("forward kinematics", "kAUTO_CONTROLLER_V", -1.0);

    kENCODER_REVS_PER_WHEEL_REV = reader.GetReal("forward kinematics", "kENCODER_REVS_PER_WHEEL_REV", -1.0);

    kTOLERANCE_POS = reader.GetReal("forward kinematics", "kTOLERANCE_POS", -1.0);
    kTOLERANCE_HEADING = reader.GetReal("forward kinematics", "kTOLERANCE_HEADING", -1.0);

    kSTRAFE_MULTIPLIER = reader.GetReal("forward kinematics", "kSTRAFE_MULTIPLIER", -1.0);

    //LIFT STUFF
    kFOOT_SPARK_ID = reader.GetInteger("lift", "kFOOT_SPARK_ID", -1);
    kLEG_LEADER_TALON_ID = reader.GetInteger("lift", "kLEG_LEADER_TALON_ID", -1);
    kLEG_FOLLOWER_TALON_ID = reader.GetInteger("lift", "kLEG_FOLLOWER_TALON_ID", -1);
}