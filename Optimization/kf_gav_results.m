%% View results of Gavin's Kalman Filter

series='gyro_y';
set=5;

seriesID = sprintf('%s_s%u',series,set);
command = sprintf('echo %0.4f %0.4f %0.4f %0.4f %s | kf_gav.exe',...
    Q_optimized(1,1),Q_optimized(1,2),Q_optimized(2,1),Q_optimized(2,2),seriesID);
[status,cmdout] = system(command);

results = importdata('output.txt');
rawdata = importdata(sprintf('idealFilterOutput\\%s_s%u_data.txt',series,set));

t = (1:1500)';
figure(2);clf;
plot(t, rawdata, '-k'); grid on; hold on;
plot(t, results, '-r','LineWidth',2);
title('KF Optimization Results: Derivative Projection model');
xlabel('t (count)'); ylabel('Sensor value');


