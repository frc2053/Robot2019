/**
 * \file InterpolatingMap.h
 * \brief Allows us to store Interpolating Values as a key value pair
 * \author FRC 2481
 */

#pragma once

#include <map>
#include "InverseInterpolable.h"
#include "Interpolable.h"

template <class K, class V>
class InterpolatingMap : public std::map<K, V> {
public:
	/**
	 * Constructor
	 * @param maximumSize max size for our map
	 */
	InterpolatingMap(size_t maximumSize) {
		m_max = maximumSize;
	}

	/**
	 * Default Constructor with size 0
	 */
	InterpolatingMap() {
		m_max = 0;
	}

	/**
	 * Put our Key Value Pair into the map
	 * @param key
	 * @param value
	 * @return return our value
	 */
	V put(K key, V value) {
		if(m_max > 0 && m_max <= std::map<K, V>::size()) {
			auto first = std::map<K, V>::begin();
			std::map<K, V>::erase(first);
		}
		std::map<K, V>::emplace(key, value);
		return value;
	}

	/**
	 * Get our interpolated value based on our key
	 * @param key
	 * @return value
	 */
	V getInterpolated(K key) {
		auto gotVal = std::map<K, V>::find(key);
		if(gotVal == std::map<K, V>::end()) {
			auto topBound = std::map<K, V>::upper_bound(key);
			auto bottomBound = std::map<K, V>::lower_bound(key);
			bottomBound--;

			if(topBound == std::map<K, V>::end() && bottomBound == std::map<K, V>::end()) {
				return std::map<K, V>::end()->second;
			}
			else if(topBound == std::map<K, V>::end()) {
				return bottomBound->second;
			}
			else if(bottomBound == std::map<K, V>::end()) {
				return topBound->second;
			}

			V topElem = topBound->second;
			V bottomElem = bottomBound->second;
			return bottomElem.interpolate(topElem, bottomBound->first.inverseInterpolate(topBound->first, key));
		}
		else {
			return gotVal->second;
		}
	}
private:
	size_t  m_max;
};
