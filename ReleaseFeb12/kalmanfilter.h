/**
 * @file kalmanfilter.h  Some settings for the kalman filter
 *
 * @author Gavin Fielder
 * @date 2/12/2018
 *
 */
 #ifndef KALMANFILTER_H
 #define KALMANFILTER_H

 #include "matrix.h"
 #include <stdint.h>

//Define constants to be passed into the kalman filter
//These values are used to index into an array of structs
#define KF_AXIS_ACCEL_X 0
#define KF_AXIS_ACCEL_Y 1
#define KF_AXIS_ACCEL_Z 2
#define KF_AXIS_GYRO_Y 3
#define KF_AXIS_GYRO_Z 4



//For setting which axis the kalman filter operates on
//The axis is set with one of these structs
typedef struct setAxis_vector_struct {
    const mat2* R;
    const mat2* Q;
    vec2* prev;
    vec2* prev_data;
    mat2* prev_covar;
} setAxis_vector;


/**
 * Performs the kalman filter
 *
 * @param  axis    The axis specifier.
 *                 options:
 *                      KF_AXIS_ACCEL_X
  *                     KF_AXIS_ACCEL_Y
  *                     KF_AXIS_ACCEL_Z
  *                     KF_AXIS_GYRO_Y
  *                     KF_AXIS_GYRO_Z
 * @param  data  the sensor reading input
 */
float kalmanFilter(uint8_t axis, int16_t data);


 #endif


