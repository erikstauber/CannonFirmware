/*
 * MathUtils.h
 *
 *  Created on: Jun 26, 2024
 *      Author: eriks
 */

#ifndef INC_MATHUTILS_H_
#define INC_MATHUTILS_H_

#include <math.h>

template <class T>
T MathUtilsGetRandomNumber(T min, T max) {
	T scale = (T) rand() / (T) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}


#endif /* INC_MATHUTILS_H_ */
