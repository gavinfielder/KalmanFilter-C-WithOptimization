#include <stdio.h>
#include <stdlib.h>

#include "matrix.h"
#include "kalmanfilter.h"

//test function prototypes
void printMatrix(mat3 A);
void printVector(vec3 v);

int main() {
    /*
    //Test matrix library
    mat3 A = {1.2774,-1.8408,-1.5143,-1.1441,-0.6154,2.1173,1.5302,-1.7691,1.4741};
    mat3 B = {-0.4145,0.1937,1.9760,2.7511,-0.0922,-1.0656,-1.8278,1.8276,0.4488};
    vec3 x = {1.1191,2.5242,2.9656};
    vec3 y = {-2.0347,-3.5122,-3.6430};

    printf("A*B = \n");
    printMatrix(multiply_MatMat(A,B));

    printf("B*A = \n");
    printMatrix(multiply_MatMat(B,A));

    printf("A*x = \n");
    printVector(multiply_MatVec(A,x));

    printf("B*y = \n");
    printVector(multiply_MatVec(B,y));

    printf("inv(A) = \n");
    printMatrix(inv(A));

    printf("inv(B) = \n");
    printMatrix(inv(B));

    printf("A+B = \n");
    printMatrix(sum_MatMat(A,B));

    printf("x+y = \n");
    printVector(sum_VecVec(x,y));

    printf("A-B = \n");
    printMatrix(diff_MatMat(A,B));

    printf("x-y = \n");
    printVector(diff_VecVec(x,y));

    printf("B-A = \n");
    printMatrix(diff_MatMat(B,A));

    printf("y-x = \n");
    printVector(diff_VecVec(y,x));

    printf("transpose(A) = \n");
    printMatrix(transpose(A));
    */


    FILE* fin;
    FILE* fout;
    int data;
    int i;
    float filtered_data;

    fin = fopen("data.txt","r");
    fout = fopen("c_output.txt","w");

    //for reading
    //fscanf(fin,"%d",&data);
    //for writing
    //fprintf(fout, "This is testing for fprintf...\n");
    //fprintf(fout,"The first data is: %d",data);

    for(i = 0; i < 1500; i++) {
        fscanf(fin,"%d",&data); // get data
        //printf("data received: %d\n",data); //debug print
        filtered_data = kalmanFilter('x',(int16_t)data);
        //printf("filtered: %f\n",filtered_data); //debug print
        fprintf(fout,"%f\n",filtered_data);
    }

    fclose(fin);
    fclose(fout);

    return 0;
}

void printMatrix(mat3 A) {
    printf("\t%f\t%f\t%f\n",A.a11,A.a12,A.a13);
    printf("\t%f\t%f\t%f\n",A.a21,A.a22,A.a23);
    printf("\t%f\t%f\t%f\n",A.a31,A.a32,A.a33);
}

void printVector(vec3 v) {
    printf("\t%f\n",v.b1);
    printf("\t%f\n",v.b2);
    printf("\t%f\n",v.b3);
}
