/**
 * \file RigidTransform2D.cpp
 * \brief A data structure to represent 2D transformations
 * \author FRC Team 2481
 */

#include "RigidTransform2D.h"

/**
 *  Inits a default Delta of 0's
 */
RigidTransform2D::Delta::Delta() : m_dx(0), m_dy(0), m_dtheta(0), m_dt(0) {

}

/**
 * Inits a 2d transform with the default constructors from Translation and Rotation
 */
RigidTransform2D::RigidTransform2D() : m_translation(Translation2D()), m_rotation(Rotation2D()) {

}

/**
 * Makes a transform from an existing translation and rotation
 * @param translation reference to translation
 * @param rotation reference to rotation
 */
RigidTransform2D::RigidTransform2D(const Translation2D &translation, const Rotation2D &rotation) : m_translation(translation), m_rotation(rotation) {

}

/**
 * Copys a transform
 * @param other
 */
RigidTransform2D::RigidTransform2D(const RigidTransform2D &other) : m_translation(other.m_translation), m_rotation(other.m_rotation) {

}

/**
 * Destructor
 */
RigidTransform2D::~RigidTransform2D() {

}

/**
 * Make a transform from just a translation
 * @param translation reference to translation
 * @return a new stack variable Transform
 */
RigidTransform2D RigidTransform2D::fromTranslation(const Translation2D &translation) {
	return RigidTransform2D(translation, Rotation2D());
}

/**
 * Make a transform from just a rotation
 * @param rotation reference to rotation
 * @return a new stack variable Transform
 */
RigidTransform2D RigidTransform2D::fromRotation(const Rotation2D &rotation) {
	return RigidTransform2D(Translation2D(), rotation);
}

/**
 * Makes a transform from a Delta Velocity
 * @param delta snippet of velocity information (in dt time)
 * @return a stack variable Transform
 */
RigidTransform2D RigidTransform2D::fromVelocity(Delta delta) {
	double sinTheta = sin(delta.GetTheta());
	double cosTheta = cos(delta.GetTheta());
	double s, c;
	if(fabs(delta.GetTheta()) < 1E-9) {
		s = 1.0 - 1.0 / 6.0 * delta.GetTheta() * delta.GetTheta();
		c = .5 * delta.GetTheta();
	}
	else {
		s = sinTheta / delta.GetTheta();
		c = (1.0 - cosTheta) / delta.GetTheta();
	}
	return RigidTransform2D(Translation2D(delta.GetX() * s - delta.GetY() * c, delta.GetX() * c + delta.GetY() * s), Rotation2D(cosTheta, sinTheta, false));
}

/**
 * Gets Translation portion of Transform
 * @return reference to translation var
 */
Translation2D& RigidTransform2D::getTranslation() {
	return m_translation;
}

/**
 * Gets Rotation portion of Transform
 * @return reference to rotation var
 */
Rotation2D& RigidTransform2D::getRotation() {
	return m_rotation;
}

void RigidTransform2D::setRotation(const Rotation2D& rotation) {
	m_rotation = rotation;
}

void RigidTransform2D::setTranslation(const Translation2D& translation) {
	m_translation = translation;
}

/**
 * Transform one transform by another
 * @param other the variable to transform by
 * @return a stack variable of a new Transform
 */
RigidTransform2D RigidTransform2D::transformBy(const RigidTransform2D &other) {
	return RigidTransform2D(m_translation.translateBy(other.m_translation.rotateBy(m_rotation)), m_rotation.rotateBy(other.m_rotation));
}

/**
 * Inverts the transform
 * @return new stack variable of inverted transform
 */
RigidTransform2D RigidTransform2D::inverse() {
	Rotation2D rotation_inversed = m_rotation.inverse();
	return RigidTransform2D(m_translation.inverse().rotateBy(rotation_inversed), rotation_inversed);
}

/**
 * Allows to move between to transforms smoothly
 * @param other end point
 * @param x rate of interpolation
 * @return one step of the interpolation
 */
RigidTransform2D RigidTransform2D::interpolate(const RigidTransform2D &other, double x) const {
	if (x <= 0) {
		return RigidTransform2D(*this);
	}
	else if (x >= 1) {
		 return RigidTransform2D(other);
	}
	return RigidTransform2D(m_translation.interpolate(other.m_translation, x), m_rotation.interpolate(other.m_rotation, x));
}

double RigidTransform2D::Delta::GetX() {
	return m_dx * m_dt;
}

double RigidTransform2D::Delta::GetY() {
	return m_dy * m_dt;
}

double RigidTransform2D::Delta::GetTheta() {
	return m_dtheta * m_dt;
}

double RigidTransform2D::Delta::GetDx() {
	return m_dx;
}

double RigidTransform2D::Delta::GetDy() {
	return m_dy;
}

double RigidTransform2D::Delta::GetDTheta() {
	return m_dtheta;
}

double RigidTransform2D::Delta::GetDt() {
	return m_dt;
}

/**
 * Make a delta manually based of off position
 * @param x X component
 * @param y Y component
 * @param theta Yaw Component
 * @param dt timestep
 * @return a stack variable Delta
 */
RigidTransform2D::Delta RigidTransform2D::Delta::fromDelta(double x, double y, double theta, double dt) {
	return Delta(x / dt, y / dt, theta / dt, dt);
}

/**
 * Makes a delta based off of velocity
 * @param dx X velocity
 * @param dy Y velocity
 * @param dtheta yaw rate
 * @param dt time step
 * @return a stack variable Delta
 */
RigidTransform2D::Delta RigidTransform2D::Delta::fromVelocity(double dx, double dy, double dtheta, double dt) {
	return Delta(dx, dy, dtheta, dt);
}

/**
 * Constructs a delta based off of velocitys
 * @param dx X Velocity
 * @param dy Y Velocity
 * @param dtheta Yaw Rate
 * @param dt time step
 */
RigidTransform2D::Delta::Delta(double dx, double dy, double dtheta, double dt) : m_dx(dx), m_dy(dy), m_dtheta(dtheta), m_dt(dt) {

}







