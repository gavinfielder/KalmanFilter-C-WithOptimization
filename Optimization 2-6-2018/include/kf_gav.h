/**
 * @file kalmanfilter.h  Some settings for the kalman filter
 *
 * @author Gavin Fielder
 * @date 12/18/2017
 *
 */
 #ifndef KF_GAV_H
 #define KF_GAV_H

 #include "matrix-cpp.h"
 #include <stdint.h>

//Define constants to be passed into the kalman filter
#define KF_AXIS_ACCEL_X 0
#define KF_AXIS_ACCEL_Y 1
#define KF_AXIS_ACCEL_Z 2
#define KF_AXIS_GYRO_Y 3
#define KF_AXIS_GYRO_Z 4




typedef struct setAxis_vector_struct {
    const mat2* R;
    const mat2* Q;
    vec2* prev;
    vec2* prev_data;
    mat2* prev_covar;
} setAxis_vector;

extern mat2 Q;

/**
 * Performs the kalman filter
 *
 * @param  axis    The axis specifier.
 *                 options: KF_AXIS_ACCEL_X, KF_AXIS_ACCEL_Y, etc.
 * @param  data_L  the low byte of the sensor reading input
 * @param  data_H  the high byte of the sensor reading input
 */
float kalmanFilter(uint8_t axis, int16_t data);


 #endif


