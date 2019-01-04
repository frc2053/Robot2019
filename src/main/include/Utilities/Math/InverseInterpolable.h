/**
 * \file InverseInterpolable
 * \brief Allows us to interpolate using a bound and query
 * \author FRC 2481
 */

#pragma once
template <class T>
class InverseInterpolable {
public:
	virtual double inverseInterpolate(const T &upper, const T &query) const = 0 ;
};
