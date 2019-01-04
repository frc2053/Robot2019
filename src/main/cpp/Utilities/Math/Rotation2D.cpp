/**
 * \file Rotation2D.cpp
 * \brief A data structure to represent 2D rotations
 * \author FRC Team 2481
 */

#include "Rotation2D.h"

/**
 * The default constructor inits a rotation with its cos component as 1
 * and sin component as 0. This is equivalent to a unit vector.
 */
Rotation2D::Rotation2D() : m_cos(1), m_sin(0) {

}

/**
 * Creates a rotation vector given x and y and if we want it normalized.
 * @param x inits cos component
 * @param y inits sin component
 * @param normalized if we want to change the length to 1
 */
Rotation2D::Rotation2D(double x, double y, bool normalized) : m_cos(x), m_sin(y) {
	if(normalized) {
		normalize();
	}
}

/**
 * Constructor to make sure we can set thing equal to eachother
 * @param other the rotation we want to copy into this one
 */
Rotation2D::Rotation2D(const Rotation2D &other) : m_cos(other.m_cos), m_sin(other.m_sin) {

}

/**
 * Destructor
 */
Rotation2D::~Rotation2D() {

}

/**
 * Operator overloading to let us copy Rotation2Ds into eachother
 * @param other the other vector we want to copy into this one
 * @return returns a reference to this rotation vector
 */
Rotation2D& Rotation2D::operator=(Rotation2D other) {
	m_cos = other.m_cos;
	m_sin = other.m_sin;
	return *this;
}

/**
 * Lets us convert an angle in radians to a rotation2D.
 * @param angle_radians angle in radians we want to represent
 * @return a new instance of Rotation2D with the angle passed in
 */
Rotation2D Rotation2D::fromRadians(double angle_radians) {
	return Rotation2D(cos(angle_radians), sin(angle_radians), false);
}

/**
 * Lets us convert an angle in degrees to a rotation2D.
 * @param angle_degrees angle in degrees we want to represent
 * @return a new instance of Rotation2D with the angle passed in
 */
Rotation2D Rotation2D::fromDegrees(double angle_degrees) {
	return fromRadians(angle_degrees * (M_PI / 180));
}

/**
 * Allows us to make sure the hypotenuse that forms with the sin
 * and cos component is of length 1 or less if it is too small set sin
 * and cos components to respectively.
 */
void Rotation2D::normalize() {
	double magnitude = hypot(m_cos, m_sin);
	if(magnitude > kEpsilon) {
		m_sin = m_sin / magnitude;
		m_cos = m_cos / magnitude;
	}
	else {
		m_sin = 0;
		m_cos = 1;
	}
}

void Rotation2D::setCos(double x) {
	m_cos = x;
}

void Rotation2D::setSin(double y) {
	m_sin = y;
}

double Rotation2D::getCos() const{
	return m_cos;
}

double Rotation2D::getSin() const{
	return m_sin;
}

double Rotation2D::getTan() const {
	if (m_cos < kEpsilon) {
		if (m_sin >= 0.0) {
			return std::numeric_limits<double>::infinity();
		}
		else {
			return -std::numeric_limits<double>::infinity();
		}
	}
	return m_sin / m_cos;
}

double Rotation2D::getRadians() const{
	return atan2(m_sin, m_cos);
}

double Rotation2D::getDegrees() const{
	return getRadians() * 180 / M_PI;
}

/**
 * Rotates a rotation2d by another one.
 * @param other how much we want to rotate by
 * @return a rotation2d that is the two rotations combined
 */
Rotation2D Rotation2D::rotateBy(const Rotation2D &other)const {
	return Rotation2D(m_cos * other.m_cos - m_sin * other.m_sin,
		m_cos * other.m_sin + m_sin * other.m_cos, true);
}

/**
 * Inverts the rotation
 * @return a rotation2d that is inverted
 */
Rotation2D Rotation2D::inverse()const {
	return Rotation2D(m_cos, -m_sin, false);
}

/**
 * Allows us to move our rotation data between two values at a rate of x
 * @param other the end rotation to go to
 * @param x rotation rate
 * @return the stepped rotation value
 */
Rotation2D Rotation2D::interpolate(const Rotation2D &other, double x)const
{
	if (x <= 0) {
		return Rotation2D(*this);
	}
	else if (x >= 1) {
		return Rotation2D(other);
	}
	double angle_diff = inverse().rotateBy(other).getRadians();
	return this->rotateBy(fromRadians(angle_diff * x));
}
