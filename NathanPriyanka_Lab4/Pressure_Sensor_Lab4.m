clear; clc;

%Nathan Chen
%E80 Lab 4
%Pressure sensor graph
%Depth and measured voltage-
depth = [0 10 20 30 40];
voltage = [1.1 1.35 1.6 1.9 2.2];
%Depth vs Voltage calibration curve
[p, S] = polyfit(depth, voltage, 1);
depthFit = linspace(min(depth), max(depth), 200);
[voltageFit, delta] = polyval(p, depthFit, S);
%Plot
figure;
hold on;
grid on;
scatter(depth, voltage, 70, 'filled');
plot(depthFit, voltageFit);
fill([depthFit fliplr(depthFit)], [voltageFit-delta fliplr(voltageFit+delta)], 'blue', 'FaceAlpha', 0.25, 'EdgeColor', 'none');
xlabel('Depth (cm)');
ylabel('Voltage (V)');
title('Pressure Sensor Calibration Curve');
legend('Measured Data', 'Linear Fit', 'Uncertainty');