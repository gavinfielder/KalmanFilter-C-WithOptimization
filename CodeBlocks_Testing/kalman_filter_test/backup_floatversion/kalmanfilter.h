/**
 * @file matrix.h Lists some 3x3 matrix and vector structures and algorithms
 *
 * @author Gavin Fielder
 * @date 12/8/2017
 *
 */
 #ifndef KALMANFILTER_H
 #define KALMANFILTER_H

#include <stdint.h>

/**
 * Performs the kalman filter
 *
 * @param  axis  'x', 'y', or 'z'
 * @param  data  the sensor reading input
 */
float kalmanFilter(char axis, int16_t data);


#endif // KALMANFILTER_H

