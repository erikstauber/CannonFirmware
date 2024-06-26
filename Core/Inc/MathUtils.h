/*
 * MathUtils.h
 *
 *  Created on: Jun 26, 2024
 *      Author: eriks
 */

#ifndef INC_MATHUTILS_H_
#define INC_MATHUTILS_H_

#include <math.h>
#include <algorithm>

template <class T>
T MathUtilsGetRandomNumber(T min, T max) {
	T scale = (T) rand() / (T) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}

template <class T>
T MathUtilsIIRFilter(T filtered_value, T new_value, T iir_constant) {
	iir_constant = std::clamp(iir_constant,static_cast<T>(0),static_cast<T>(1.0));
	return filtered_value*(1.0-iir_constant) + new_value*iir_constant;
}


#endif /* INC_MATHUTILS_H_ */
