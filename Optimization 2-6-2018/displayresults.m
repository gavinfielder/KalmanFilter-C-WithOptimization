clear;clc;
data = importdata('data.txt');
output = importdata('c_output.txt');

t=(1:1500)';

figure(1);clf;
plot(t, data,'-k','MarkerSize',3); hold on; grid on;
plot(t, output, '-r', 'MarkerSize',3);
