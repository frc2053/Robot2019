/**
 * \file InterpolatingDouble.cpp
 * \brief A double that can be interpolated
 * \author FRC 2481
 */

#include "InterpolatingDouble.h"

/**
 * Constructor for our Interpolating double
 * @param val initial value
 */
InterpolatingDouble::InterpolatingDouble(double val) {
	m_value = val;
}

/**
 * Interpolate between this and other at rate x
 * @param other our end point
 * @param x rate of interpolation
 * @return one step of our interpolation later
 */
InterpolatingDouble InterpolatingDouble::interpolate(const InterpolatingDouble &other, double x) const {
	double dydx = other.m_value - m_value;
	double searchY = dydx * x + m_value;
	return InterpolatingDouble(searchY);
}

/**
 * Inverse of interpolate
 * @param upper upper bound
 * @param query
 * @return
 */
double InterpolatingDouble::inverseInterpolate(const InterpolatingDouble &upper, const InterpolatingDouble &query) const {
	double upper_to_lower = upper.m_value - m_value;
	if(upper_to_lower <= 0) {
		return 0;
	}
	double query_to_lower = query.m_value - m_value;
	if(query_to_lower <= 0) {
		return 0;
	}
	return query_to_lower / upper_to_lower;
}

/**
 * Allows us to compare values
 * @param lhs one side
 * @param rhs other side
 * @return true if lhs < rhs
 */
bool operator < (const InterpolatingDouble &lhs, const InterpolatingDouble &rhs) {
	return lhs.m_value < rhs.m_value;
}

/**
 * Checks for equality
 * @param other
 * @return true if they are equal
 */
bool InterpolatingDouble::operator == (const InterpolatingDouble &other) {
	return m_value == other.m_value;
}
