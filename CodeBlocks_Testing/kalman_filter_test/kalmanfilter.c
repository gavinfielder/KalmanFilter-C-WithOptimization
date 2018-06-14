 /**
 * @file kalmanfilter.c  Implements the bike project kalman filter
 *                       for the accelerometer sensor readings
 *
 * @author Gavin Fielder
 * @date 12/18/2017
 *
 */
#include <stdint.h>
#include <math.h>
#include "matrix.h"
#include "kalmanfilter.h"


//Process noise covariance matrices
const mat2 Q_accel_x = {
    23.063380343677736,-1.613290434952044,2.268308223798982,-0.024613452149723};

const mat2 Q_accel_y = {
    23.063380343677736,-1.613290434952044,2.268308223798982,-0.024613452149723};

const mat2 Q_accel_z = {
    23.063380343677736,-1.613290434952044,2.268308223798982,-0.024613452149723};

const mat2 Q_gyro_y = {
    1.643063571443009,0.018611354586077,0.006152842293845,-4.853573659635199e-05};

const mat2 Q_gyro_z = {
    1.643063571443009,0.018611354586077,0.006152842293845,-4.853573659635199e-05};

// Measurement noise covariance matrices
const mat2 R_accel_x = {
          3467.94939915498,          3496.60560195686,
          3496.60560195686,          6993.40893217699 };

const mat2 R_accel_y = {
       2612.5,       2806.2,
       2806.2,       5610.1 };

const mat2 R_accel_z = {
	     4484.7,       3784.7,
       3784.7,       7571.5 };

const mat2 R_gyro_y = {
	     82.184,       73.821,
       73.821,       148.09 };

const mat2 R_gyro_z = {
       49.734,       45.409,
       45.409,       90.772 };


//Predictive matrix
const mat2 Ap = {
     1,   1,
	   0,   1  };
//Predictive matrix A transpose - predefined for speed
const mat2 A_T = {
     1,   0,
	   1,   1 };

//Identity matrix
const mat2 I = {
	   1,   0,
	   0,   1   };

// file-scope variable declarations - holds the previous estimated
// state and covariance matrix of each axis
static vec2 prev_accel_x = {0,0}; //state, 1st deriv
static vec2 prev_accel_y = {0,0}; //state, 1st deriv
static vec2 prev_accel_z = {0,0}; //state, 1st deriv
static vec2 prev_gyro_y = {0,0}; //state, 1st deriv
static vec2 prev_gyro_z = {0,0}; //state, 1st deriv
static vec2 prev_data_accel_x = {0,0}; //reading, 1st deriv
static vec2 prev_data_accel_y = {0,0}; //reading, 1st deriv
static vec2 prev_data_accel_z = {0,0}; //reading, 1st deriv
static vec2 prev_data_gyro_y = {0,0}; //reading, 1st deriv
static vec2 prev_data_gyro_z = {0,0}; //reading, 1st deriv
static mat2 prev_covar_accel_x = {1,0,0,1}; //identity initial
static mat2 prev_covar_accel_y = {1,0,0,1}; //identity initial
static mat2 prev_covar_accel_z = {1,0,0,1}; //identity initial
static mat2 prev_covar_gyro_y = {1,0,0,1}; //identity initial
static mat2 prev_covar_gyro_z = {1,0,0,1}; //identity initial

// define axis structs
//This is the array that the axis setting uses
//It allows setting all 5 pointers simultaneously, for speed.
// Copied: from kalmanfilter.h :
    //#define KF_AXIS_ACCEL_X 0
    //#define KF_AXIS_ACCEL_Y 1
    //#define KF_AXIS_ACCEL_Z 2
    //#define KF_AXIS_GYRO_Y 3
    //#define KF_AXIS_GYRO_Z 4
setAxis_vector ax_arr[5] = {
  {&R_accel_x, &Q_accel_x, &prev_accel_x, &prev_data_accel_x, &prev_covar_accel_x },
  {&R_accel_y, &Q_accel_y, &prev_accel_y, &prev_data_accel_y, &prev_covar_accel_y },
  {&R_accel_z, &Q_accel_z, &prev_accel_z, &prev_data_accel_z, &prev_covar_accel_z },
  {&R_gyro_y,  &Q_gyro_y,  &prev_gyro_y,  &prev_data_gyro_y,  &prev_covar_gyro_y },
  {&R_gyro_z,  &Q_gyro_z,  &prev_gyro_z,  &prev_data_gyro_z,  &prev_covar_gyro_z }
};


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
//tmpReturn_t kalmanFilter(uint8_t axis, uint8_t data_L, uint8_t data_H) {
float kalmanFilter(uint8_t axis, int16_t data) {

        //Use axis setting to set all the necessary pointers
        const setAxis_vector ax = ax_arr[axis];

		//Other declarations
		vec2 predicted_state;
		vec2 estimated_state;
		mat2 predicted_covar;
		mat2 K; //kalman gain
		vec2 z; //data vector

		// Begin predictive update step
		predicted_state = multiply_MatVec(Ap,*(ax.prev));
		predicted_covar = sum_MatMat(multiply_MatMat(multiply_MatMat(Ap,*(ax.prev_covar)),A_T),*(ax.Q));

		//Begin measurement update step
		// Compute kalman gain
		K = multiply_MatMat(predicted_covar, inv(sum_MatMat(predicted_covar,*(ax.R))));
        // Derive a data vector including the first and second derivatives
		z.b1 = (float) data; //the new data point
		z.b2 = z.b1 - (*(ax.prev_data)).b1; //first derivative
		//Now let's compute the filtered state estimate
		estimated_state = sum_VecVec(predicted_state, multiply_MatVec(K,diff_VecVec(z,predicted_state)));

		//Now to calculate the covariance for the next iteration
		*(ax.prev_covar) = multiply_MatMat(diff_MatMat(I,K),predicted_covar);
		//Update prev_state for the next iteration
		(*(ax.prev)).b1 = estimated_state.b1;
		(*(ax.prev)).b2 = estimated_state.b2;
		//Log data for the derivative calculations next cycle
        (*(ax.prev_data)).b1 = z.b1;
		(*(ax.prev_data)).b2 = z.b2;

        return estimated_state.b1;
}


uint32_t convertForUART(float num) {
    tmpReturn_t temp;
    temp.value = (uint32_t) abs((int)num);
    //determine sign
    if (num < 0) temp.signBit = 1;
    else temp.signBit = 0;

    if (signBit == 1) {
        //2's complement
        temp.value = ~(temp.value) + 1;
    }

    return temp.value;
}

int16_t concatenateUint8(uint8_t low, uint8_t high) {
    uint16_t result_value;
    uint16_t concatenated = ((uint16_t) low) + (((uint16_t) high) << 8);
    if (concatenated & 0x8000) {
        //is negative
        result_value = ~concatenated + 1;
        return ((int16_t) result_value * -1);
    }
    else {
        result_value = concatenated;
        return result_value;
    }
}














