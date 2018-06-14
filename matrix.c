 /**
 * @file matrix.h Implements some 2x2 matrix/vector structures and algorithms
 * 
 * @author Gavin Fielder
 * @date 12/18/2017
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
 vec2 multiply_MatVec(mat2 A, vec2 b) {
	 vec2 r;
	 r.b1 = A.a11*b.b1 + A.a12*b.b2;
	 r.b2 = A.a21*b.b1 + A.a22*b.b2;
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
 mat2 multiply_MatMat(mat2 A, mat2 B) {
	 mat2 r;
	 
	 r.a11 = A.a11*B.a11 + A.a12*B.a21;
	 r.a12 = A.a11*B.a12 + A.a12*B.a22;
	 
	 r.a21 = A.a21*B.a11 + A.a22*B.a21;
	 r.a22 = A.a21*B.a12 + A.a22*B.a22;
	 
	 return r;
 }
 
  /**
  * Returns the transpose of a matrix
	* 
	* @param  A  the matrix
	*
	* @return  the transpose of A
	*/
	mat2 transpose(mat2 A) {
		mat2 R;
		R.a11 = A.a11;
		R.a12 = R.a21;
		
		R.a21 = A.a12;
		R.a22 = A.a22;
 
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
mat2 sum_MatMat(mat2 A, mat2 B) {
	mat2 R;
	R.a11 = A.a11 + B.a11;
	R.a12 = A.a12 + B.a12;
	R.a21 = A.a21 + B.a21;
	R.a22 = A.a22 + B.a22;
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
vec2 sum_VecVec(vec2 a, vec2 b) {
	vec2 r;
	r.b1 = a.b1 + b.b1;
	r.b2 = a.b2 + b.b2;
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
vec2 diff_VecVec(vec2 a, vec2 b) {
	vec2 r;
	r.b1 = a.b1 - b.b1;
	r.b2 = a.b2 - b.b2;
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
mat2 diff_MatMat(mat2 A, mat2 B) {
	mat2 R;
	R.a11 = A.a11 - B.a11;
	R.a12 = A.a12 - B.a12;
	R.a21 = A.a21 - B.a21;
	R.a22 = A.a22 - B.a22;
	return R;
	
}

 /**
  * Returns the inverse of a matrix
	* 
	* @param  A  the matrix
	*
	* @return  the inverse of A
	*/
mat2 inv(mat2 A) {
	float D;
	mat2 R;
  D = A.a11*A.a22 - A.a21*A.a12;
	R.a11 = A.a22 / D;
	R.a12 = -A.a12 / D;
	R.a21 = -A.a21 / D;
  R.a22 = A.a11 / D;	
	return R;
}
