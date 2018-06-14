 /**
 * @file kalmanfilter.c  Implements the bike project kalman filter
 *                       for the accelerometer sensor readings
 *
 * @author Gavin Fielder
 * @date 12/18/2017
 *
 */
#include <stdint.h>
#include "matrix-cpp.h"
#include "kf_choi.h"
#include <math.h> //for exp() in Phi matrix calc


//Process noise covariance matrices
mat2 Q_choi = {
          4312.5478,         0,
         0,        4312.5478 };


// Measurement noise covariance matrices
float R_accel_x_choi = 4312.5478;
float R_accel_y_choi = 4312.5478;
float R_accel_z_choi = 4312.5478;
float R_gyro_y_choi = 4312.5478;
float R_gyro_z_choi = 4312.5478;


//Predictive matrix
mat2 Ap_choi;
//Predictive matrix A transpose - predefined for speed
mat2 A_T_choi;

//Identity matrix
const mat2 I_choi = {
	   1,   0,
	   0,   1   };
/* We're going to assume H is identity
//Output transformation to state estimation
const mat3 H = {
	   1,   0,
	   0,   1 }; */
const vec2 H_choi = {1, 0}; //note this H is its own transpose




// file-scope variable declarations - holds the previous estimated
// state and covariance matrix of each axis
static vec2 prev_accel_x_choi =  {0,0};
static vec2 prev_accel_y_choi =  {0,0};
static vec2 prev_accel_z_choi =  {0,0};
static vec2 prev_gyro_y_choi =  {0,0};
static vec2 prev_gyro_z_choi =  {0,0};
static float prev_data_accel_x_choi =  0;
static float prev_data_accel_y_choi =  0;
static float prev_data_accel_z_choi =  0;
static float prev_data_gyro_y_choi = 0;
static float prev_data_gyro_z_choi = 0;
static mat2 prev_covar_accel_x_choi = {1,0,0,1}; //identity initial
static mat2 prev_covar_accel_y_choi = {1,0,0,1}; //identity initial
static mat2 prev_covar_accel_z_choi = {1,0,0,1}; //identity initial
static mat2 prev_covar_gyro_y_choi = {1,0,0,1}; //identity initial
static mat2 prev_covar_gyro_z_choi = {1,0,0,1}; //identity initial



// define axis structs
setAxis_vector_choi ax_arr_choi[5] = {
  {&R_accel_x_choi, &Q_choi, &prev_accel_x_choi, &prev_data_accel_x_choi, &prev_covar_accel_x_choi },
  {&R_accel_y_choi, &Q_choi, &prev_accel_y_choi, &prev_data_accel_y_choi, &prev_covar_accel_y_choi },
  {&R_accel_z_choi, &Q_choi, &prev_accel_z_choi, &prev_data_accel_z_choi, &prev_covar_accel_z_choi },
  {&R_gyro_y_choi,  &Q_choi,  &prev_gyro_y_choi,  &prev_data_gyro_y_choi,  &prev_covar_gyro_y_choi },
  {&R_gyro_z_choi,  &Q_choi,  &prev_gyro_z_choi,  &prev_data_gyro_z_choi,  &prev_covar_gyro_z_choi }
};


/**
 * Performs the kalman filter
 *
 * @param  axis    The axis specifier.
 *                 options: KF_AXIS_ACCEL_X, KF_AXIS_ACCEL_Y, etc.
 * @param  data_L  the low byte of the sensor reading input
 * @param  data_H  the high byte of the sensor reading input
 */
float kalmanFilter_choi(uint8_t axis, int16_t data) {

        //Use axis setting to set all the necessary pointers
        const setAxis_vector_choi ax = ax_arr_choi[axis];

		//Other declarations
		vec2 predicted_state;
		vec2 estimated_state;
		mat2 predicted_covar;
		vec2 K; //kalman gain
		float z; //data
		float r;

		// Begin predictive update step
		predicted_state = multiply_MatVec(Ap_choi,*(ax.prev)); //x~(k)
		predicted_covar = sum_MatMat(multiply_MatMat(multiply_MatMat(Ap_choi,*(ax.prev_covar)),A_T_choi),*(ax.Q));


		//Begin measurement update step
        // Derive a data value
		z = (float) data; //the new data point
		// Compute kalman gain
		r = z - predicted_state.b1; //predicted_state.b1 is same as H*predicted_state
		K = multiply_VecScale(multiply_MatVec(predicted_covar, H_choi), (1.0/(multiply_RowCol(multiply_RowMat(H_choi, predicted_covar),H_choi)+(*(ax.R)))));

		//Now let's compute the filtered state estimate
		estimated_state = sum_VecVec(predicted_state, multiply_VecScale(K, r));

		//Now to calculate the covariance for the next iteration
		*(ax.prev_covar) = multiply_MatMat(diff_MatMat(I_choi,multiply_ColRow(K,H_choi)),predicted_covar);
		//Update prev_state for the next iteration
		(*(ax.prev)).b1 = estimated_state.b1;
		(*(ax.prev)).b2 = estimated_state.b2;
		//Log data for the derivative calculations next cycle
        (*(ax.prev_data)) = z;

        return estimated_state.b1;
}














