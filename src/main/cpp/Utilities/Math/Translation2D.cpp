/**
 * \file Translation2D.cpp
 * \brief A data structure to represent 2D vectors
 * \author FRC Team 2481
 */

#include "Utilities/Math/Translation2D.h"
#include <cmath>

/**
 * The default constructor sets up an origin vector
 */
Translation2D::Translation2D()
	: m_x(0), m_y(0) {
}

/**
 * Creates a vector with x and y lengths
 * @param x x length
 * @param y y length
 */
Translation2D::Translation2D(double x, double y)
	: m_x(x), m_y(y) {
}

/**
 * Constructor to set one translation to another
 * @param other
 */
Translation2D::Translation2D(const Translation2D &other)
	: m_x(other.m_x), m_y(other.m_y) {
}

/**
 * Destruct Me!
 */
Translation2D::~Translation2D()
{
}

/**
 * Sets x length
 * @param x X length
 */
void Translation2D::setX(double x)
{
	m_x = x;
}

/**
 * Sets y length
 * @param y Y length
 */
void Translation2D::setY(double y)
{
	m_y = y;
}

/**
 * Gets X Value
 * @return X Value
 */
double Translation2D::getX() const
{
	return m_x;
}

/**
 * Gets Y Value
 * @return Y Value
 */
double Translation2D::getY() const
{
	return m_y;
}

/**
 * Distance between two points in space
 * @return the distance between the x and y components of the vector
 */
double Translation2D::norm() const
{
	return sqrt(m_x * m_x + m_y * m_y);
}

/**
 * Reversed direction vector
 * @return (-x, -y)
 */
Translation2D Translation2D::inverse() const
{
	return Translation2D(-m_x, -m_y);
}

/**
 * Moves a vector by an amount passed in
 * @param other the vector to translate by
 * @return A new vector that has been translated
 */
Translation2D Translation2D::translateBy(const Translation2D &other) const
{
	return Translation2D(m_x + other.m_x, m_y + other.m_y);
}


/**
 * Rotates a translation by a Rotation
 *
 * -----> by 45 ==
 *
 * \
 *  \
 *   >
 *
 * @param rotation
 * @return
 */
Translation2D Translation2D::rotateBy(const Rotation2D &rotation) const
{
	return Translation2D(m_x * rotation.getCos() - m_y * rotation.getSin(), m_y * rotation.getCos() + m_x * rotation.getSin());
}


/**
 *
 * Allows us to move our translation along another translation at rate X
 *
 * @param other end goal of interp
 * @param x rate of rotation
 * @return new interpolated Translation
 */
Translation2D Translation2D::interpolate(const Translation2D &other, double x) const {
	if (x <= 0) {
		return Translation2D(*this);
	}
	else if (x >= 1) {
		return Translation2D(other);
	}
	return extrapolate(other, x);
}


/**
 * Gives us predicted data of translation vector given another vector
 * @param other vector to extrapolate to
 * @param x rate
 * @return extrapolated vector
 */
Translation2D Translation2D::extrapolate(const Translation2D &other, double x) const {
	return Translation2D(x * (other.m_x - m_x) + m_x, x * (other.m_y - m_y) + m_y);
}
