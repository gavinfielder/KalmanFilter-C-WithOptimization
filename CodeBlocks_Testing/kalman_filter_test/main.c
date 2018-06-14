#include <stdio.h>
#include <stdlib.h>

#include "matrix.h"
#include "kalmanfilter.h"

int main() {

    uint8_t num1_L = 0x04;
    uint8_t num1_H = 0x21; //dec 8452
    uint8_t num2_L = 0x36;
    uint8_t num2_H = 0xFF; //dec -458
    uint8_t num3_L = 0x11;
    uint8_t num3_H = 0x1D; //dec 7441
    uint8_t num4_L = 0x32;
    uint8_t num4_H = 0xDA; //dec -9678

    int16_t num1= concatenateUint8(num1_L, num1_H);
    int16_t num2= concatenateUint8(num2_L, num2_H);
    int16_t num3= concatenateUint8(num3_L, num3_H);
    int16_t num4= concatenateUint8(num4_L, num4_H);


    printf("num1=%p\n",num1);
    printf("num2=%p\n",num2);
    printf("num3=%p\n",num3);
    printf("num4=%p\n",num4);

/*
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
        filtered_data = kalmanFilter(KF_AXIS_ACCEL_X,(int16_t)data);
        //printf("filtered: %f\n",filtered_data); //debug print
        fprintf(fout,"%f\n",filtered_data);
    }

    fclose(fin);
    fclose(fout);
*/
    return 0;
}
