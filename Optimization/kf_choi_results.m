%% View results of Dr. Choi's Kalman Filter

seriesID = sprintf('%s_s%u',series,set);
command = sprintf('echo %0.4f %0.4f %0.1f %s | kf_choi.exe',...
    inputs_optimized(1),inputs_optimized(2),inputs_optimized(3),...
    seriesID);
[status,cmdout] = system(command);

results = importdata('output.txt');
rawdata = importdata(sprintf('idealFilterOutput\\%s_s%u_data.txt',series,set));

t = (1:1500)';
figure(1);clf;
plot(t, rawdata, '-k'); grid on; hold on;
plot(t, results, '-r','LineWidth',2);
title('KF Optimization Results: Choi model');
xlabel('t (count)'); ylabel('Sensor value');
