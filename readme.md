# Kalman Filter in C with Optimization

This is an updated (2/23/2018) version of the first two kalman filter algorithms tested with the unmanned bicycle project at CSU Chico. These two kalman filter algorithms are:
 - Gavin Fielder's derivative-projection predictive model
 - Dr. Sugki Choi's model (adapted for C code by Gavin Fielder)

Included in this repo is an automated optimization script that used matlab to find the optimum settings for the process noise covariance matrix for both Gavin and Dr. Choi's kalman filter. It does this by comparing results against an optimal filter (25-point centered moving median of data) and returning the sum squares error across 5 tests through the shell to matlab which refines the values with the fminsearch function. 

Last updated 2/23/2018
