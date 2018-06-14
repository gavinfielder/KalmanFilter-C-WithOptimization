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
#include "kalmanfilter_choi.h"
#include <math.h> //for exp() in Phi matrix calc


//Process noise covariance matrices
const mat2 Q_accel_x = {
          4312.5478,         0,
         0,        4312.5478 };

const mat2 Q_accel_y = {
	    11.304,      0.19044,
      0.19044,     0.012137 };

const mat2 Q_accel_z = {
	    22.692,      0.31481,
      0.31481,     0.015708 };

const mat2 Q_gyro_y = {
	   0.5818,      0.022344,
     0.022344,    0.0039857 };

const mat2 Q_gyro_z = {
	   0.36394,     0.015996,
     0.015996,    0.0033498 };

// Measurement noise covariance matrices
const float R_accel_x = 4312.5478;
const float R_accel_y = 0;
const float R_accel_z = 0;
const float R_gyro_y = 0;
const float R_gyro_z = 0;


//Predictive matrix
const float tau = 1.0;
//static float phi_value_1 = 1 - exp(-tau);
//const float phi_value_1 = 0.63;
#define phi_value_1 0.6
const mat2 Ap = {
        phi_value_1, 0,
        0, phi_value_1};
//Predictive matrix A transpose - predefined for speed
const mat2 A_T = {
        phi_value_1, 0,
        0, phi_value_1};
//static mat2 Phi = {phi_value_1, 0, 0, phi_value_1};

//Identity matrix
const mat2 I = {
	   1,   0,
	   0,   1   };
/* We're going to assume H is identity
//Output transformation to state estimation
const mat3 H = {
	   1,   0,
	   0,   1 }; */
const vec2 H = {1, 0}; //note this H is its own transpose




// file-scope variable declarations - holds the previous estimated
// state and covariance matrix of each axis
static vec2 prev_accel_x =  {0,0};
static vec2 prev_accel_y =  {0,0};
static vec2 prev_accel_z =  {0,0};
static vec2 prev_gyro_y =  {0,0};
static vec2 prev_gyro_z =  {0,0};
static float prev_data_accel_x =  0;
static float prev_data_accel_y =  0;
static float prev_data_accel_z =  0;
static float prev_data_gyro_y = 0;
static float prev_data_gyro_z = 0;
static mat2 prev_covar_accel_x = {1,0,0,1}; //identity initial
static mat2 prev_covar_accel_y = {1,0,0,1}; //identity initial
static mat2 prev_covar_accel_z = {1,0,0,1}; //identity initial
static mat2 prev_covar_gyro_y = {1,0,0,1}; //identity initial
static mat2 prev_covar_gyro_z = {1,0,0,1}; //identity initial



// define axis structs
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
		vec2 K; //kalman gain
		float z; //data
		float r;

		// Begin predictive update step
		predicted_state = multiply_MatVec(Ap,*(ax.prev)); //x~(k)
		predicted_covar = sum_MatMat(multiply_MatMat(multiply_MatMat(Ap,*(ax.prev_covar)),A_T),*(ax.Q));


		//Begin measurement update step
        // Derive a data value
		z = (float) data; //the new data point
		// Compute kalman gain
		r = z - predicted_state.b1; //predicted_state.b1 is same as H*predicted_state
		K = multiply_VecScale(multiply_MatVec(predicted_covar, H), (1.0/(multiply_RowCol(multiply_RowMat(H, predicted_covar),H)+(*(ax.R)))));

		//Now let's compute the filtered state estimate
		estimated_state = sum_VecVec(predicted_state, multiply_VecScale(K, r));

		//Now to calculate the covariance for the next iteration
		*(ax.prev_covar) = multiply_MatMat(diff_MatMat(I,multiply_ColRow(K,H)),predicted_covar);
		//Update prev_state for the next iteration
		(*(ax.prev)).b1 = estimated_state.b1;
		(*(ax.prev)).b2 = estimated_state.b2;
		//Log data for the derivative calculations next cycle
        (*(ax.prev_data)) = z;

        return estimated_state.b1;
}














