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

//DRIVEBASE CALIBRATION
constexpr int kDRIVEBASE_FL_CAL = 3015;
constexpr int kDRIVEBASE_BL_CAL = 2463;
constexpr int kDRIVEBASE_FR_CAL = 4030;
constexpr int kDRIVEBASE_BR_CAL = 579;
constexpr int kDRIVEBASE_TICKS_PER_REV = 4096;

//CONVERSION FACTORS
constexpr double kINCHESPERWEELREV = 7.85; //inches
constexpr int kTICKSPERREVOFENCODER = 8192;
constexpr int kENCODERREVPERWHEELREV = 3.2;

//DRIVEBASE PHYSICAL PARAMS
constexpr double kDRIVEBASE_WIDTH = 26.249; //inches
constexpr double kDRIVEBASE_LENGTH = 21; //inches

//DRIVEBASE TALONS
constexpr int kDRIVETALON_FL_DRIVE_ID = 2;
constexpr int kDRIVETALON_FR_DRIVE_ID = 3;
constexpr int kDRIVETALON_BL_DRIVE_ID = 4;
constexpr int kDRIVETALON_BR_DRIVE_ID = 5;

//FORWARD KINEMATICS
constexpr double kFORWARD_KINEMATICS_WEIGHT = 0;
constexpr double KGYRO_WEIGHT = 1;

//AUTO CONFIG
constexpr double kAUTO_P = .5;
constexpr double kAUTO_I = 0;
constexpr double kAUTO_D = 0;
constexpr double kAUTO_V = .06;
constexpr double kCONTROLLER_PERIOD = 0.05;

//STRING CONSTS
const std::string FLMODULEID = "FrontLeftModule";
const std::string FRMODULEID = "FrontRightModule";
const std::string BLMODULEID = "BackLeftModule";
const std::string BRMODULEID = "BackRightModule";

const std::string FLDRIVEID = "FrontLeftDrive";
const std::string FRDRIVEID = "FrontRightDrive";
const std::string BLDRIVEID = "BackLeftDrive";
const std::string BRDRIVEID = "BackRightDrive";

