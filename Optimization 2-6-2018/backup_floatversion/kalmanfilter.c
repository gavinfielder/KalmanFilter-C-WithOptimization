 /**
 * @file kalmanfilter.c  Implements the bike project kalman filter
 *                       for the accelerometer sensor readings
 *
 * @author Gavin Fielder
 * @date 12/8/2017
 *
 */
#include <stdint.h>
#include "matrix.h"

//Process noise covariance matrix
 const mat3 Q = {
     15.9685814250449,   0.695411178052061,    0.0410414837001868,
     0.695411178052061,  0.119519598872807,    0.0147804810716697,
     0.0410414837001867,  0.0147804810716697,  0.00413678470913156  };

// Measurement noise covariance matrices
// For some reason the baseline appears to be different for each axis
const mat3 Rx = {
     3467.94939915498,   -3496.803337336,    3502.36514654214,
     -3496.803337336,    6993.40893217699,   -10505.3615619302,
     3502.36514654214,   -10505.3615619302,  20920.3186906826  };

const mat3 Ry = {
     2612.48227707361,   -2803.91528574605,  2822.84203691349,
     -2803.91528574605,  5610.14871069603,   -8435.32133244386,
     2822.84203691349,   -8435.32133244386,  16757.3040942851    };

const mat3 Rz = {
    4484.7368539026,     -3786.77872715143,   3119.97323015343,
    -3786.77872715143,   7571.46758461197,    -10697.5677936402,
    3119.97323015343,    -10697.5677936402,   21377.1728281076    };

//Predictive matrix
const mat3 Ap = {
     1,   1,   0,
	   0,   1,   1,
	   0,   0,   1   };
//Predictive matrix A transpose - predefined for speed
const mat3 A_T = {
     1,   0,   0,
	   1,   1,   0,
	   0,   1,   1   };

//Identity matrix
const mat3 I = {
	   1,   0,   0,
	   0,   1,   0,
	   0,   0,   1   };

/* We're going to assume H is identity
//Output transformation to state estimation
const mat3 H = {
	   1,   0,   0,
	   0,   1,   0,
	   0,   0,   1   }; */

/**
 * Performs the kalman filter
 *
 * @param  axis  'x', 'y', or 'z'
 * @param  data  the sensor reading input
 *
 * @return  the estimate state
 */
float kalmanFilter(char axis, int16_t data) {
	  //Initialize static records of previous state of each axis
		static vec3 prev_x = {132,0,0}; //state, 1st deriv, 2nd deriv
		static vec3 prev_y = {0,0,0}; //state, 1st deriv, 2nd deriv
		static vec3 prev_z = {0,0,0}; //state, 1st deriv, 2nd deriv
		static vec3 prev_data_x = {132, 0, 0};
		static vec3 prev_data_y = {0, 0, 0};
		static vec3 prev_data_z = {0, 0, 0};
		static mat3 prev_covar_x = {1,0,0,0,1,0,0,0,1}; //identity initial
		static mat3 prev_covar_y = {1,0,0,0,1,0,0,0,1}; //identity initial
		static mat3 prev_covar_z = {1,0,0,0,1,0,0,0,1}; //identity initial
		//Other declarations
		vec3 predicted_state;
		vec3 estimated_state;
		mat3 predicted_covar;
		mat3 K; //kalman gain
		vec3 z; //data vector

		//Set pointers to relevant axis
		vec3* prev_state;
		vec3* prev_data;
		mat3* prev_covar;
		const mat3* R;
		switch(axis - 'x') {
			case 0:
              prev_state = &prev_x;
			  prev_covar = &prev_covar_x;
			  prev_data = &prev_data_x;
			  R = &Rx;
			  break;
			case 1:
              prev_state = &prev_y;
			  prev_covar = &prev_covar_y;
			  prev_data = &prev_data_y;
			  R = &Ry;
			  break;
			case 2:
              prev_state = &prev_z;
			  prev_covar = &prev_covar_z;
			  prev_data = &prev_data_z;
			  R = &Rz;
			  break;
		}

		// Begin predictive update step
		predicted_state = multiply_MatVec(Ap,*prev_state);
		predicted_covar = sum_MatMat(multiply_MatMat(multiply_MatMat(Ap,*prev_covar),A_T),Q);

		//Begin measurement update step
		// Compute kalman gain
		K = multiply_MatMat(predicted_covar, inv(sum_MatMat(predicted_covar,*R)));
        // Derive a data vector including the first and second derivatives
		z.b1 = (float) data; //the new data point
		z.b2 = z.b1 - (*prev_data).b1; //first derivative
		z.b3 = z.b2 - (*prev_data).b2; //second derivative
		//Now let's compute the filtered state estimate
		estimated_state = sum_VecVec(predicted_state, multiply_MatVec(K,diff_VecVec(z,predicted_state)));

		//Now to calculate the covariance for the next iteration
		*prev_covar = multiply_MatMat(diff_MatMat(I,K),predicted_covar);
		//Update prev_state for the next iteration
		(*prev_state).b1 = estimated_state.b1;
		(*prev_state).b2 = estimated_state.b2;
		(*prev_state).b3 = estimated_state.b3;
		//Log data for the derivative calculations next cycle
        (*prev_data).b1 = z.b1;
		(*prev_data).b2 = z.b2;
		(*prev_data).b3 = z.b3;

        return estimated_state.b1;
}














