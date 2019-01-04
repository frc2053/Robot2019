#include "TigerSwerve.h"
#include "../../RobotMap.h"
#include "../Math/RigidTransform2D.h"

/**
 * Constructor. Sets up all the talons and makes sure we are ready to drive
 * @param talons A map of shared_ptrs which are references to all the talons needed to control the drivebase
 */
TigerSwerve::TigerSwerve(std::unordered_map<std::string, std::shared_ptr<can::TalonSRX>>& talons) {
	xAxis = 0;
	yAxis = 0;
	rotAxis = 0;
	currentYaw = 0;
	modules = std::make_shared<std::unordered_map<std::string, SwerveModule>>();
	modules->insert(std::make_pair(FLMODULEID, SwerveModule(talons.find(FLDRIVEID)->second, talons.find(FLROTID)->second)));
	modules->insert(std::make_pair(FRMODULEID, SwerveModule(talons.find(FRDRIVEID)->second, talons.find(FRROTID)->second)));
	modules->insert(std::make_pair(BLMODULEID, SwerveModule(talons.find(BLDRIVEID)->second, talons.find(BLROTID)->second)));
	modules->insert(std::make_pair(BRMODULEID, SwerveModule(talons.find(BRDRIVEID)->second, talons.find(BRROTID)->second)));
}

/**
 * Default Destructor
 */
TigerSwerve::~TigerSwerve() {

}

/**
 * The main command that controls the math of how to rotate the wheels given joystick input
 * @param xSpeed the horizontal magnitude of the joystick
 * @param ySpeed the vertical magnitude of the joystick
 * @param rotSpeed the horizontal magnitude of the rotation joystick
 * @param headingOffset angle in degrees that we are offset by. Usually if we start a match sideways this will be non zero
 */
void TigerSwerve::Drive(double xSpeed, double ySpeed, double rotSpeed, double headingOffset) {
	rotSpeed = rotSpeed * 0.05;
	Translation2D trans(ySpeed, xSpeed);
	Rotation2D rot = Rotation2D::fromDegrees(rotSpeed);
	Rotation2D gyroAngle = Rotation2D::fromDegrees(-headingOffset);
	currentYaw = headingOffset;
	trans = trans.rotateBy(gyroAngle);

	double flWheelSpeed;
	double frWheelSpeed;
	double blWheelSpeed;
	double brWheelSpeed;
	Rotation2D flWheelAngle;
	Rotation2D frWheelAngle;
	Rotation2D blWheelAngle;
	Rotation2D brWheelAngle;

	SwerveInverseKinematics(trans, rotSpeed, frWheelSpeed, flWheelSpeed, brWheelSpeed, blWheelSpeed, flWheelAngle, frWheelAngle, blWheelAngle, brWheelAngle);
	modules->find(FLMODULEID)->second.Set(flWheelSpeed, flWheelAngle, true);
	modules->find(FRMODULEID)->second.Set(frWheelSpeed, frWheelAngle, true);
	modules->find(BLMODULEID)->second.Set(blWheelSpeed, blWheelAngle, true);
	modules->find(BRMODULEID)->second.Set(brWheelSpeed, brWheelAngle, true);
}

/**
 * This sets all the wheels to "interlock" to make us harder to push
 * TODO: Currently does nothing.
 */
void TigerSwerve::SetBrakeMode() {
	for(int i = 0; i < (signed) modules->size(); i++) {
		//modules->at(i).Set(modules->at(i).GetLocation().GetAngle(), 0);
	}
}

/**
 * Utility function for converting degrees to radians
 * @param deg degrees
 * @return radians of input
 */
double TigerSwerve::deg2rad(double deg) {
	return deg * M_PI / 180.0;
}

/**
 * Helper function to drive with no gyro
 * @param x X value of joystick
 * @param y Y value of joystick
 * @param rotation X value of rotation joystick
 */
void TigerSwerve::DriveRobotOriented(double x, double y, double rotation) {
	Drive(x, y, rotation, 0);
}

/**
 * Helper function to drive with a gyro
 * @param x X value of joystick
 * @param y Y value of joystick
 * @param rotation X value of rotation joystick
 * @param gyro the current angle of the gyroscope
 */
void TigerSwerve::DriveFieldOriented(double x, double y, double rotation, double gyro) {
	Drive(x, y, rotation, gyro);
}

/**
 * Returns the collection of modules
 * @return a shared_ptr to a map of swerve modules
 */
std::shared_ptr<std::unordered_map<std::string, SwerveModule>> TigerSwerve::GetModules() {
	return modules;
}

/**
 * This function does the math to make sure we rotate the wheels the correct way and drive them the right way
 * Make sure to double check this because this is not default....
 * @param velocity x and y components of the joystick input
 * @param yawRate rotation rate
 * @param wheelSpeedFR ref to wheelSpeed for front right
 * @param wheelSpeedFL ref to wheelSpeed for front left
 * @param wheelSpeedBR ref to wheelSpeed for back right
 * @param wheelSpeedBL ref to wheelSpeed for back left
 * @param wheelAngleFL ref to wheelAngle for front left
 * @param wheelAngleFR ref to wheelAngle for front right
 * @param wheelAngleBL ref to wheelAngle for back left
 * @param wheelAngleBR ref to wheelAngle for back right
 */
void TigerSwerve::SwerveInverseKinematics(Translation2D &velocity,
		double yawRate, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
		Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR) {

	double A = velocity.getX() - yawRate * (kDRIVEBASE_LENGTH / 2.0);
	double B =  velocity.getX() + yawRate * (kDRIVEBASE_LENGTH / 2.0);
	double C = velocity.getY() - yawRate * (kDRIVEBASE_WIDTH / 2.0);
	double D = velocity.getY() + yawRate * (kDRIVEBASE_WIDTH / 2.0);

	wheelSpeedFL = sqrt(pow(A, 2) + pow(D, 2));
	wheelSpeedFR = sqrt(pow(B, 2) + pow(C, 2));
	wheelSpeedBL = sqrt(pow(B, 2) + pow(D, 2));
	wheelSpeedBR = sqrt(pow(A, 2) + pow(C, 2));

	wheelAngleFL = Rotation2D(A, C, true);
	wheelAngleFR = Rotation2D(B, C, true);
	wheelAngleBL = Rotation2D(A, D, true);
	wheelAngleBR = Rotation2D(B, D, true);

	double maxWheelSpeed = std::max(wheelSpeedFL, std::max(wheelSpeedFR, std::max(wheelSpeedBL, wheelSpeedBR)));
	if(maxWheelSpeed > 1) {
		wheelSpeedFL /= maxWheelSpeed;
		wheelSpeedFR /= maxWheelSpeed;
		wheelSpeedBL /= maxWheelSpeed;
		wheelSpeedBR /= maxWheelSpeed;
	}
}

/**
 * Allows us to calculate where the robot is based on its wheel velocities
 * @param flAngle ref to front left module angle
 * @param flVelocity ref to front left module velocity
 * @param frAngle ref to front right module angle
 * @param frVelocity ref to front right module velocity
 * @param blAngle ref to back left module angle
 * @param blVelocity ref to back left module velocity
 * @param brAngle ref to back right module angle
 * @param brVelocity ref to back right module velocity
 * @return a stack variable Delta that holds velocity data
 */
RigidTransform2D::Delta TigerSwerve::SwerveForwardKinematics(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle, RigidTransform2D::Delta& frVelocity,
		Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity, Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity) {

	double VyFL = flVelocity.GetX() * flAngle.getCos();
	double VxFL = flVelocity.GetX() * flAngle.getSin();

	double VyFR = frVelocity.GetX() * frAngle.getCos();
	double VxFR = frVelocity.GetX() * frAngle.getSin();

	double VyBL = blVelocity.GetX() * blAngle.getCos();
	double VxBL = blVelocity.GetX() * blAngle.getSin();

	double VyBR = brVelocity.GetX() * brAngle.getCos();
	double VxBR = brVelocity.GetX() * brAngle.getSin();

	double VxBack = (VxBR + VxBL) / 2.0;
	double VxFront = (VxFR + VxFL) / 2.0;
	double VyRight = (VyFR + VyBR) / 2.0;
	double VyLeft = (VyFL + VyBL) / 2.0;

	double yawRate1 = (VxBack - VxFront) / kDRIVEBASE_LENGTH;
	double yawRate2 = (VyRight - VyLeft) / kDRIVEBASE_WIDTH;
	double yawRate = (yawRate1 + yawRate2) / 2.0;

	double Vxc1 = VxBack - yawRate * kDRIVEBASE_LENGTH / 2.0;
	double Vxc2 = VxFront + yawRate * kDRIVEBASE_LENGTH / 2.0;
	double Vyc1 = VyRight - yawRate * kDRIVEBASE_WIDTH / 2.0;
	double Vyc2 = VyLeft + yawRate * kDRIVEBASE_WIDTH / 2.0;
	double Vxc = (Vxc1 + Vxc2) / 2.0;
	double Vyc = (Vyc1 + Vyc2) / 2.0;

	return RigidTransform2D::Delta::fromDelta(Vxc, Vyc, yawRate, flVelocity.GetDt());
}
























