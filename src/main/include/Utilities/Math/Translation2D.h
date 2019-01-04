/**
 * \file Translation2D.h
 * \brief Represents a 2d vector
 * \author FRC Team 2841
 *
 */

#pragma once

#include "Rotation2D.h"

/**
 * This class represents a two dimensional vector.
 * We can set, translate and rotate as well as interpolate and extrapolate.
 */
class Translation2D
{
public:
	Translation2D();
	Translation2D(double x, double y);
	Translation2D(const Translation2D &other);
	virtual ~Translation2D();

	void setX(double x);
	void setY(double y);

	double getX()const;
	double getY()const;

	double norm() const;
	Translation2D inverse() const;

	Translation2D translateBy(const Translation2D &other) const;
	Translation2D rotateBy(const Rotation2D &rotation)const;

	virtual Translation2D interpolate(const Translation2D &other, double x)const;
	Translation2D extrapolate(const Translation2D &other, double x)const;

private:
	double m_x;
	double m_y;
};
