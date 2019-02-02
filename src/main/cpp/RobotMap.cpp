#include "INIReader.h"
#include "RobotMap.h"



RobotMap::RobotMap() {

}

void RobotMap::Init() {
    INIReader reader("/home/lvuser/robotprops.ini");
    //CONTROLLER STUFF
    kDRIVER_CONTROLLER_PORT = reader.GetInteger("controller", "kDRIVERCONTROLLERPORT", -1);
    std:: cout << kDRIVER_CONTROLLER_PORT;
    kOPERATOR_CONTROLLER_PORT = reader.GetInteger("controller", "kOPERATORCONTROLLERPORT", -1);
    std:: cout << kOPERATOR_CONTROLLER_PORT;
    kCONTROLLER_PERIOD = reader.GetReal("controller", "kCONTROLLERPERIOD", -1.0);
    std:: cout << kCONTROLLER_PERIOD;
    
    //ENCODER STUFF
    kTICKS_PER_REV_OF_ENCODER = reader.GetInteger("encoder", "TICKSPERREVOFENCODER", -1);
    kTICKS_PER_REV_NEO = reader.GetInteger("encoder", "TICKSPERREVNEO", -1);
    kELEVATORTICKS_PER_INCH = reader.GetInteger("encoder", "kELEVATORTICKSPERINCH", -1);
    kWRIST_GEAR_RATIO = reader.GetReal("encoder", "kWRISTGEARRATIO", -1.0);
    kFLAPPER_GEAR_RATIO = reader.GetReal("encoder", "kFLAPPERGEARRATIO", -1.0);
    kWHEEL_DIAMETER = reader.GetReal("encoder", "kWHEELDIAMETER", -1.0);
    kDRIVE_GEAR_RATIO = reader.GetReal("encoder", "kDRIVEGEARRATIO", -1.0);
    kWHEEL_BASE_LENGTH = reader.GetReal("encoder", "kWHEELBASELENGTH", -1.0);
    kWHEEL_BASE_WIDTH = reader.GetReal("encoder", "kWHEELBASEWIDTH", -1);

    //DRIVEBASE ROTATION PARAMETERS
    kROTATION_P = reader.GetReal("drivebase rotation", "kROTATIONP", -1.0);
    kROTATION_I = reader.GetReal("drivebase rotation", "kROTATIONI", -1.0);
    kROTATION_D = reader.GetReal("drivebase rotation", "kROTATIOND", -1.0);
    kROTATION_ANGLE_TOLERANCE = reader.GetReal("drivebase rotation", "kROTATIONANGLETOLERANCE", -1.0);

    //ELEVATOR PID
    kELEVATOR_F = reader.GetReal("elevator pid", "kROTATIONF", -1.0);
    kELEVATOR_P = reader.GetReal("elevator pid", "kROTATIONP", -1.0);
    kELEVATOR_I = reader.GetReal("elevator pid", "kROTATIONI", -1.0);
    kELEVATOR_D = reader.GetReal("elevator pid", "kROTATIOND", -1.0);

    //DRIVEBASE TALONS
    kDRIVESPARK_FL_ID = reader.GetReal("drivebase talons", "kDRIVESPARKFLID", -1);
    kDRIVESPARK_FR_ID = reader.GetReal("drivebase talons", "kDRIVESPARKFRID", -1);
    kDRIVESPARK_BL_ID = reader.GetReal("drivebase talons", "kDRIVESPARKBLID", -1);
    kDRIVESPARK_BR_ID = reader.GetReal("drivebase talons", "kDRIVESPARKBRID", -1);

    //INTAKE TALONS
    kINTAKE_WHEELS_ID = reader.GetInteger("intake talons", "kDRIVESPARKFLID", -1);
    kINTAKE_ACTUATOR_ID = reader.GetInteger("intake talons", "kDRIVESPARKACTUATORID", -1);

    //ELEVATOR TALONS
    kELEVATOR_LEADER_ID = reader.GetInteger("elevator talons", "kELEVATORLEADERID", -1);
    kELEVATOR_FOLLOWERONE_ID = reader.GetInteger("elevator talons", "kELEVATORFOLLOWERONEID", -1);
    kELEVATOR_FOLLOWERTWO_ID = reader.GetInteger("elevator talons", "kELEVATORFOLLOWERTWO", -1);

    //FLAPPER TALONS
    kINTAKE_FLAPPER_LEFT_ID = reader.GetInteger("flapper talons", "kINTAKEFLAPPERLEFTID", -1);
    kINTAKE_FLAPPER_RIGHT_ID = reader.GetInteger("flapper talons", "kINTAKEFLAPPERRIGHTID", -1);

    //INTAKE PARAMS
    kINTAKE_ANGLE_BALL = reader.GetReal("intake params", "kINTAKEANGLEBALL", -1.0);
    kINTAKE_ANGLE_UP = reader.GetReal("intake params", "kINTAKEANGLEUP", -1.0);
    kFLAPPER_UP_ANGLE = reader.GetReal("intake params", "kFLAPPERUPANGLE", -1.0);
    kFLAPPER_DOWN_ANGLE = reader.GetReal("intake params", "kFLAPPERDOWNANGLE", -1.0);
    //;)
    kINTAKE_ANGLE_TOLERANCE = reader.GetInteger("intake params", "kINTAKEANGLETOLERANCE", -1);
    kINTAKE_SPEED = reader.GetReal("intake params", "kINTAKESPEED", -1.0);

    //ELEVATOR PARAMS
    kELEVATOR_GROUND = reader.GetReal("elevator params", "kELEVATORGROUND", -1.0);
    kELEVATOR_LEVEL_ONE_HATCH = reader.GetReal("elevator params", "kELEVATORLEVELONEHATCH", -1.0);
    kELEVATOR_LEVEL_TWO_HATCH = reader.GetReal("elevator params", "kELEVATORLEVELTWOHATCH", -1.0);
    kELEVATOR_LEVEL_THREE_HATCH = reader.GetReal("elevator params", "kELEVATORLEVELTHREEHATCH", -1.0);
    kELEVATOR_LEVEL_ONE_PORT = reader.GetReal("elevator params", "kELEVATORLEVELONEPORT", -1.0);
    kELEVATOR_LEVEL_TWO_PORT = reader.GetReal("elevator params", "kELEVATORLEVELTWOPORT", -1.0);
    kELEVATOR_LEVEL_THREE_PORT = reader.GetReal("elevator params", "kELEVATORLEVELTHREEPORT", -1.0);

    //FORWARD KINEMATICS
    kFORWARD_KINEMATICS_WEIGHT = reader.GetReal("forward kinematics", "kFORWARDKINEMATICSWEIGHT", -1.0);
    kGYRO_WEIGHT = reader.GetReal("forward kinematics", "kGYROWEIGHT", -1.0);
    kAUTO_CONTROLLER_P = reader.GetReal("forward kinematics", "kAUTOCONTROLLERP", -1.0);
    kAUTO_CONTROLLER_I = reader.GetReal("forward kinematics", "kAUTOCONTROLLERI", -1.0);
    kAUTO_CONTROLLER_D = reader.GetReal("forward kinematics", "kAUTOCONTROLLERD", -1.0);
    kAUTO_CONTROLLER_V = reader.GetReal("forward kinematics", "kAUTOCONTROLLERV", -1.0);

    kENCODER_REVS_PER_WHEEL_REV = reader.GetReal("forward kinematics", "kENCODERREVSPERWHEELREV", -1.0);

    kTOLERANCE_POS = reader.GetReal("forward kinematics", "kTOLERANCEPOS", -1.0);
    kTOLERANCE_HEADING = reader.GetReal("forward kinematics", "kTOLERANCEHEADING", -1.0);

    kSTRAFE_MULTIPLIER = reader.GetReal("forward kinematics", "kSTRAFEMULTIPLIER", -1.0);

    //LIFT STUFF
    kFOOT_SPARK_ID = reader.GetInteger("lift", "kfOOTSPARKID", -1);
    kLEG_LEADER_TALON_ID = reader.GetInteger("lift", "kLEGLEADERTALONID", -1);
    kLEG_FOLLOWER_TALON_ID = reader.GetInteger("lift", "kLEGFOLLOWERTALONID", -1);
}