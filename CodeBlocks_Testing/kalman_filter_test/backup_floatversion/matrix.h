/**
 * @file matrix.h Lists some 3x3 matrix and vector structures and algorithms
 *
 * @author Gavin Fielder
 * @date 12/8/2017
 *
 */
 #ifndef MATRIX_H
 #define MATRIX_H

 typedef struct mat3_struct {
	 float a11; float a12; float a13;
	 float a21; float a22; float a23;
	 float a31; float a32; float a33;
 } mat3;

 typedef struct vec3_struct {
	 float b1;
	 float b2;
	 float b3;
 } vec3;

 /**
  * Multiplies a matrix by a vector
  *
  * @param  A  the matrix
  * @param  b  the vector
  *
  * @return  the product, a vector
  */
 vec3 multiply_MatVec(mat3 A, vec3 b);

 /**
  * Multiples a matrix by a matrix
  *
  * @param A the left matrix
  * @param B the right matrix
  *
  * @return the product
  */
mat3 multiply_MatMat(mat3 A, mat3 B);

 /**
  * Returns the transpose of a matrix
	*
	* @param  A  the matrix
	*
	* @return  the transpose of A
	*/
mat3 transpose(mat3 A);


	/**
	 * Returns the sum of two matrices
	 *
	 * @param  A  the first matrix
	 * @param  B  the second matrix
	 *
	 * @return the sum of A and B
	 */
mat3 sum_MatMat(mat3 A, mat3 B);

	/**
	 * Returns the difference of two vectors
	 *
	 * @param  a  the vector to subtract from
	 * @param  b  the vector to subtract
	 *
	 * @return the difference of a and b
	 */
vec3 diff_VecVec(vec3 a, vec3 b);

	/**
	 * Returns the difference of two matrices
	 *
	 * @param  A  the matrix to subtract from
	 * @param  B  the matrix to subtract
	 *
	 * @return the difference of A and A
	 */
mat3 diff_MatMat(mat3 A, mat3 B);

 /**
  * Returns the inverse of a matrix
	*
	* @param  A  the matrix
	*
	* @return  the inverse of A
	*/
mat3 inv(mat3 A);


	/**
	 * Returns the sum of two vectors
	 *
	 * @param  a  the first vector
	 * @param  b  the second vector
	 *
	 * @return the sum of a and b
	 */
vec3 sum_VecVec(vec3 a, vec3 b);


 #endif



