 /**
 * @file matrix.h Implements some 3x3 matrix/vector structures and algorithms
 *
 * @author Gavin Fielder
 * @date 12/8/2017
 *
 */

 #include "matrix.h"

 /**
  * Multiplies a matrix by a vector
  *
  * @param  A  the matrix
  * @param  b  the vector
  *
  * @return  the product, a vector
  */
 vec3 multiply_MatVec(mat3 A, vec3 b) {
	 vec3 r;
	 r.b1 = A.a11*b.b1 + A.a12*b.b2 + A.a13*b.b3;
	 r.b2 = A.a21*b.b1 + A.a22*b.b2 + A.a23*b.b3;
   r.b3 = A.a31*b.b1 + A.a32*b.b2 + A.a33*b.b3;
	 return r;
 }

 /**
  * Multiples a matrix by a matrix
  *
  * @param A the left matrix
  * @param B the right matrix
  *
  * @return the product
  */
 mat3 multiply_MatMat(mat3 A, mat3 B) {
	 mat3 r;

	 r.a11 = A.a11*B.a11 + A.a12*B.a21 + A.a13*B.a31;
	 r.a12 = A.a11*B.a12 + A.a12*B.a22 + A.a13*B.a32;
	 r.a13 = A.a11*B.a13 + A.a12*B.a23 + A.a13*B.a33;

	 r.a21 = A.a21*B.a11 + A.a22*B.a21 + A.a23*B.a31;
	 r.a22 = A.a21*B.a12 + A.a22*B.a22 + A.a23*B.a32;
	 r.a23 = A.a21*B.a13 + A.a22*B.a23 + A.a23*B.a33;

	 r.a31 = A.a31*B.a11 + A.a32*B.a21 + A.a33*B.a31;
	 r.a32 = A.a31*B.a12 + A.a32*B.a22 + A.a33*B.a32;
	 r.a33 = A.a31*B.a13 + A.a32*B.a23 + A.a33*B.a33;

	 return r;
 }

  /**
  * Returns the transpose of a matrix
	*
	* @param  A  the matrix
	*
	* @return  the transpose of A
	*/
	mat3 transpose(mat3 A) {
		mat3 R;
		R.a11 = A.a11;
		R.a12 = A.a21;
		R.a13 = A.a31;

		R.a21 = A.a12;
		R.a22 = A.a22;
		R.a23 = A.a32;

		R.a31 = A.a13;
		R.a32 = A.a23;
		R.a33 = A.a33;

	  return R;
	}


  /**
	 * Returns the sum of two matrices
	 *
	 * @param  A  the first matrix
	 * @param  B  the second matrix
	 *
	 * @return the sum of A and B
	 */
mat3 sum_MatMat(mat3 A, mat3 B) {
	mat3 R;
	R.a11 = A.a11 + B.a11;
	R.a12 = A.a12 + B.a12;
	R.a13 = A.a13 + B.a13;
	R.a21 = A.a21 + B.a21;
	R.a22 = A.a22 + B.a22;
	R.a23 = A.a23 + B.a23;
	R.a31 = A.a31 + B.a31;
	R.a32 = A.a32 + B.a32;
	R.a33 = A.a33 + B.a33;
	return R;
}

	/**
	 * Returns the sum of two vectors
	 *
	 * @param  a  the first vector
	 * @param  b  the second vector
	 *
	 * @return the sum of a and b
	 */
vec3 sum_VecVec(vec3 a, vec3 b) {
	vec3 r;
	r.b1 = a.b1 + b.b1;
	r.b2 = a.b2 + b.b2;
	r.b3 = a.b3 + b.b3;
	return r;
}

	/**
	 * Returns the difference of two vectors
	 *
	 * @param  a  the vector to subtract from
	 * @param  b  the vector to subtract
	 *
	 * @return the difference of a and b
	 */
vec3 diff_VecVec(vec3 a, vec3 b) {
	vec3 r;
	r.b1 = a.b1 - b.b1;
	r.b2 = a.b2 - b.b2;
	r.b3 = a.b3 - b.b3;
	return r;
}

	/**
	 * Returns the difference of two matrices
	 *
	 * @param  A  the matrix to subtract from
	 * @param  B  the matrix to subtract
	 *
	 * @return the difference of A and A
	 */
mat3 diff_MatMat(mat3 A, mat3 B) {
	mat3 R;
	R.a11 = A.a11 - B.a11;
	R.a12 = A.a12 - B.a12;
	R.a13 = A.a13 - B.a13;
	R.a21 = A.a21 - B.a21;
	R.a22 = A.a22 - B.a22;
	R.a23 = A.a23 - B.a23;
	R.a31 = A.a31 - B.a31;
	R.a32 = A.a32 - B.a32;
	R.a33 = A.a33 - B.a33;
	return R;

}

 /**
  * Returns the inverse of a matrix
	*
	* @param  A  the matrix
	*
	* @return  the inverse of A
	*/
mat3 inv(mat3 A) {
	float D;
	mat3 R;
	//these values are needed more than once. Speed with temp vars.
	float tmp1 = A.a22*A.a33 - A.a23*A.a32;
	float tmp2 = A.a21*A.a33 - A.a23*A.a31;
	float tmp3 = A.a21*A.a32 - A.a22*A.a31;
	D =   A.a11*(tmp1) - A.a12*(tmp2) + A.a13*(tmp3);
	//printf("Determinant is %f\n",D);
	R.a11 = (tmp1) / D;
	R.a12 = (A.a13*A.a32 - A.a33*A.a12) / D;
	R.a13 = (A.a12*A.a23 - A.a22*A.a13) / D;
	R.a21 = (-tmp2) / D;
	R.a22 = (A.a11*A.a33 - A.a31*A.a13) / D;
	R.a23 = (A.a13*A.a21 - A.a23*A.a11) / D;
	R.a31 = (tmp3) / D;
	R.a32 = (A.a12*A.a31 - A.a32*A.a11) / D;
	R.a33 = (A.a11*A.a22 - A.a21*A.a12) / D;
	return R;
}



