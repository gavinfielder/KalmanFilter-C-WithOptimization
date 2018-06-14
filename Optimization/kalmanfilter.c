 /**
 * @file kalmanfilter.c  Implements the bike project kalman filter
 *                       for the accelerometer sensor readings
 *
 * @author Gavin Fielder
 * @date 12/18/2017
 *
 */
#include <stdint.h>
#include "matrix.h"
#include "kalmanfilter.h"


//Process noise covariance matrices
const mat2 Q = {
          15.5931530528813,         0.240184666793283,
         0.240184666793283,        0.0136738698654096 };


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
/* We're going to assume H is identity
//Output transformation to state estimation
const mat3 H = {
	   1,   0,
	   0,   1 }; */

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
setAxis_vector ax_arr[5] = {
  {&R_accel_x, &Q, &prev_accel_x, &prev_data_accel_x, &prev_covar_accel_x },
  {&R_accel_y, &Q, &prev_accel_y, &prev_data_accel_y, &prev_covar_accel_y },
  {&R_accel_z, &Q, &prev_accel_z, &prev_data_accel_z, &prev_covar_accel_z },
  {&R_gyro_y,  &Q,  &prev_gyro_y,  &prev_data_gyro_y,  &prev_covar_gyro_y },
  {&R_gyro_z,  &Q,  &prev_gyro_z,  &prev_data_gyro_z,  &prev_covar_gyro_z }
};


/**
 * Performs the kalman filter
 *
 * @param  axis    The axis specifier.
 *                 options: KF_AXIS_ACCEL_X, KF_AXIS_ACCEL_Y, etc.
 * @param  data_L  the low byte of the sensor reading input
 * @param  data_H  the high byte of the sensor reading input
 */
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














