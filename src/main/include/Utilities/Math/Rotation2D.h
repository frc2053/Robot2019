/**
 * \file Rotation2D.h
 * \brief Represents a 2 dimensional rotation in radians
 * \author FRC Team 2841
 *
 */

#pragma once

#include <math.h>
#include <limits>

/**
 * This class represents a two dimensional rotation in radians.
 * We can interpolate, rotate, inverse get the output in degrees or
 * radians.
 */
class Rotation2D {
public:
	Rotation2D();
	Rotation2D(double x, double y, bool normalized);
	Rotation2D(const Rotation2D &other);
	virtual ~Rotation2D();

	Rotation2D& operator=(Rotation2D arg);

	static Rotation2D fromRadians(double angle_radians);
	static Rotation2D fromDegrees(double angle_degrees);
	Rotation2D rotateBy(const Rotation2D &other) const;
	Rotation2D inverse() const;

	void normalize();

	void setCos(double x);
	void setSin(double y);

	double getCos() const;
	double getSin() const;
	double getTan() const;
	double getRadians() const;
	double getDegrees() const;

	virtual Rotation2D interpolate(const Rotation2D &other, double x) const;
	Rotation2D extrapolate(const Rotation2D& other, double x) const;
private:
	double m_cos;
	double m_sin;
	const double kEpsilon = 1E-9;
};
