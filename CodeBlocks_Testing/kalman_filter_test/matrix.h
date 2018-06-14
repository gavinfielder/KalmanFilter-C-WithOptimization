/**
 * @file matrix.h Lists some 2x2 matrix and vector structures and algorithms
 *
 * @author Gavin Fielder
 * @date 12/18/2017
 *
 */
 #ifndef MATRIX_H
 #define MATRIX_H

 typedef struct mat2_struct {
	 float a11; float a12;
	 float a21; float a22;
 } mat2;

 typedef struct vec2_struct {
	 float b1;
	 float b2;
 } vec2;

 /**
  * Multiplies a matrix by a vector
  *
  * @param  A  the matrix
  * @param  b  the vector
  *
  * @return  the product, a vector
  */
 vec2 multiply_MatVec(mat2 A, vec2 b);

 /**
  * Multiples a matrix by a matrix
  *
  * @param A the left matrix
  * @param B the right matrix
  *
  * @return the product
  */
mat2 multiply_MatMat(mat2 A, mat2 B);

 /**
  * Returns the transpose of a matrix
	*
	* @param  A  the matrix
	*
	* @return  the transpose of A
	*/
mat2 transpose(mat2 A);


	/**
	 * Returns the sum of two matrices
	 *
	 * @param  A  the first matrix
	 * @param  B  the second matrix
	 *
	 * @return the sum of A and B
	 */
mat2 sum_MatMat(mat2 A, mat2 B);

	/**
	 * Returns the difference of two vectors
	 *
	 * @param  a  the vector to subtract from
	 * @param  b  the vector to subtract
	 *
	 * @return the difference of a and b
	 */
vec2 diff_VecVec(vec2 a, vec2 b);

	/**
	 * Returns the difference of two matrices
	 *
	 * @param  A  the matrix to subtract from
	 * @param  B  the matrix to subtract
	 *
	 * @return the difference of A and A
	 */
mat2 diff_MatMat(mat2 A, mat2 B);

 /**
  * Returns the inverse of a matrix
	*
	* @param  A  the matrix
	*
	* @return  the inverse of A
	*/
mat2 inv(mat2 A);


	/**
	 * Returns the sum of two vectors
	 *
	 * @param  a  the first vector
	 * @param  b  the second vector
	 *
	 * @return the sum of a and b
	 */
vec2 sum_VecVec(vec2 a, vec2 b);



/**
 * Multiplies a row vector by a column vector and returns the scalar value
 *
 * @param  r  the row vector
 * @param  c  the column vector
 *
 * @return  the product of the row and column vectors
 */
 float multiply_RowCol(vec2 r, vec2 c);

 /**
  * Multiplies a column vector by a row vector and returns the matrix
  *
  * @param c  the column vector
  * @param r  the row vector
  *
  * @return the product (matrix) of the column vector and the row vector
  */
mat2 multiply_ColRow(vec2 c, vec2 r);

/**
 * Multiplies a row vector with a matrix
 *
 * @param  r  the row vector
 * @param  A  the matrix
 *
 * @return  the row vector product
 */
vec2 multiply_RowMat(vec2 r, mat2 A);

/**
 * Multiplies a vector by a scalar
 *
 * @param  v  the vector
 * @param  k  the scalar
 *
 * @return the product
 */
vec2 multiply_VecScale(vec2 v, float k);



 #endif



