

%% Optimize Q for Dr. Choi's Kalman Filter, accel_x series
% clear;clc;
% 
% series = 'accel_x';
% 
% % Initial value:
% inputs0 = [4312; 4312; 1];
%   
% options = optimset('Display','iter','PlotFcns',@optimplotfval);
% [inputs_optimized,residual] = fminsearch(@(inp) kf_choi(inp,series),inputs0,options);
% 
% %Results 1 - without tau optimization
% % Q_optimized = [2.845895151553978e+04,0.003329717532723;-0.017872375062357,-1.102911062454112e+04]
% % J = 1.735445600000000e+09
% 
% % Results 2 - with tau optimization
% % inputs_optimized = [3.627838373926881e+03;1.566209138162446e+03;2.852146171387194]

%% Compare results
% 
% % Choi model optimization results
% inputs_optimized = [3.627838373926881e03;1.566209138162446e03;2.852146171387194];
% % Derivative projection model optimization results
% Q_optimized = [23.063380343677736,-1.613290434952044;2.268308223798982,-0.024613452149723];
% 
% % Select data set 1-5
% set=5;
% 
% kf_choi_results;
% kf_gav_results;

%% Optimize Q for Gavin's Kalman Filter, accel_x series
clear;clc;

series = 'accel_x';

% Initial value:
Q0 = [15.6,0.24;0.24,0.01367];
  
options = optimset('Display','iter','PlotFcns',@optimplotfval);
[inputs_optimized,residual] = fminsearch(@(Q) kf_gav(Q,series),Q0,options);

% Results
Q_optimized = [23.063380343677736,-1.613290434952044;2.268308223798982,-0.024613452149723]
% J = 376234300

%% Optimize Q for Gavin's Kalman Filter, gyro_y series
clear;clc;

series = 'gyro_y';

% Initial value:
Q0 = [	     0.5818,      0.022344,
     0.022344,    0.0039857 ];
  
options = optimset('Display','iter','PlotFcns',@optimplotfval);
[Q_optimized,residual] = fminsearch(@(Q) kf_gav(Q,series),Q0,options);

% Results
% Q_optimized = [1.643063571443009,0.018611354586077;0.006152842293845,-4.853573659635199e-05]
% J = 593686100



