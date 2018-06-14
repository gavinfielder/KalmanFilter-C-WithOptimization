

#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <stdint.h>
#include <iomanip>

#include "consoleInput.h"
#include "matrix.h"
#include "kf_gav.h"
#include "kf_choi.h"

using std::string;
using std::stringstream;
using std::ifstream;
using std::ofstream;
using std::endl;
using std::cin;
using std::cout;

uint8_t axis;
int16_t data[1500];
float ideal[1500];
float kf_gav[1500];
float kf_choi[1500];
float J_gav;
float J_choi;

//Function prototypes
void loadData(string series);
void performKF_gav();
void performKF_choi();

int main() {
    //Get Q
    cin >> Q_choi.a11;
    //cin >> Q_choi.a12;
    Q_choi.a12 = 0;
    //cin >> Q_choi.a21;
    Q_choi.a21 = 0;
    cin >> Q_choi.a22;

    //Get tau
    float tau;
    cin >> tau;

    //Set Phi matrix
    float phi_value_1 = 1 - exp(-tau);
    Ap_choi.a11 = phi_value_1;
    Ap_choi.a12 = 0;
    Ap_choi.a21 = 0;
    Ap_choi.a22 = phi_value_1;

    //Get series name
    string series;
    cin >> series;

    //Load Data
    loadData(series);

    //Perform Kalman Filter
    performKF_choi();

    //Return residual
    cout << J_choi << endl;

    //Output data
    ofstream fout;
    fout.open("output.txt");
    for (int i = 0; i < 1500; i++)
        fout << kf_choi[i] << endl;
    fout.close();

    return 0;
}

//Loads raw data and ideal filter output from file
void loadData(string series) {
        stringstream data_filename_s;
        data_filename_s << "idealFilterOutput/" << series << "_data.txt";
        stringstream ideal_filename_s;
        ideal_filename_s << "idealFilterOutput/" << series << "_idealfiltered.txt";
        string data_filename = data_filename_s.str();
        string ideal_filename = ideal_filename_s.str();
        ifstream fin;
        fin.open(data_filename.c_str());
        for (int i = 0; i < 1500; i++) {
            fin >> data[i];
        }
        fin.close();
        fin.open(ideal_filename.c_str());
        for (int i = 0; i < 1500; i++) {
            fin >> ideal[i];
        }
        fin.close();
        if (series.substr(0,7)=="accel_x")
            axis = KF_AXIS_ACCEL_X;
        else if (series.substr(0,7)=="accel_y")
            axis = KF_AXIS_ACCEL_Y;
        else if (series.substr(0,7)=="accel_z")
            axis = KF_AXIS_ACCEL_Z;
        else if (series.substr(0,6)=="gyro_y")
            axis = KF_AXIS_GYRO_Y;
        else if (series.substr(0,6)=="gyro_z")
            axis = KF_AXIS_GYRO_Z;
        J_gav = 0;
        J_choi = 0;
}

void performKF_gav() {
    for (int i = 0; i < 1500; i++) {
        kf_gav[i] = kalmanFilter(axis,data[i]);
        J_gav += pow((kf_gav[i] - ideal[i]),2);
    }
}

void performKF_choi() {
    for (int i = 0; i < 1500; i++) {
        kf_choi[i] = kalmanFilter_choi(axis,data[i]);
        J_choi += pow((kf_choi[i] - ideal[i]),2);
    }
}








