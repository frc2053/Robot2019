/**
 * \file Interpolable.h
 * \brief an interface that defines a interpolate function
 * \author FRC Team 2841
 *
 */

#pragma once
template <class T>
class Interpolable {
	virtual T interpolate(const T& other, double x) const = 0;
};
