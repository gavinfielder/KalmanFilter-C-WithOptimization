%% Optimize Q for Gavin's Kalman Filter, accel_x series
clear;clc;

series = 'accel_x';

% Initial value:
Q0 = [15.6,0.24;0.24,0.01367];
  
options = optimset('Display','iter','PlotFcns',@optimplotfval);
[Q_optimized,residual] = fminsearch(@(Q) kf_gav(Q,series),Q0,options);

% Results
% Q_optimized = [23.063380343677736,-1.613290434952044;2.268308223798982,-0.024613452149723]
% J = 376234300

%% Optimize Q for Dr. Choi's Kalman Filter, accel_x series
clear;clc;

series = 'accel_x';

% Initial value:
Q0 = [4312,0;0,4312];
  
options = optimset('Display','iter','PlotFcns',@optimplotfval);
[Q_optimized,residual] = fminsearch(@(Q) kf_choi(Q,series),Q0,options);

%Results
% Q_optimized = [2.845895151553978e+04,0.003329717532723;-0.017872375062357,-1.102911062454112e+04]
% J = 1.735445600000000e+09

