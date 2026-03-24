% magcalibration.m
clc; clear;

% --- Load Uncalibrated Data ---
filenum = '001';
logreader; % Assuming this loads magX, magY, and headingIMU
magXuncal = magX;
magYuncal = magY;
headingIMUuncal = (headingIMU * pi) / 180; % Use pi for better precision

% Create a time vector that matches the length of the data
fs = 10.1; % Sampling frequency based on your 10.1 divisor
t_uncal = (0:length(magXuncal)-1) / fs;

% --- Load Calibrated Data ---
filenum = '004';
logreader; 
magXcal = magX;
magYcal = magY;
headingIMUcal = (headingIMU * pi) / 180;

t_cal = (0:length(magXcal)-1) / fs;

% --- Figure 1: XY Scatter (The "Magnetometer Circle") ---
figure(1);
plot(magXuncal, magYuncal, 'r.'); % Using dots often looks better for mag data
hold on;
plot(magXcal, magYcal, 'b.');
axis equal; % CRITICAL for seeing if the calibration made the data circular
grid on;
title('Calibrated versus Uncalibrated Magnetometer Data')
legend('Uncalibrated','Calibrated')
xlabel('Induction X (mG)')
ylabel('Induction Y (mG)')

% --- Figure 2: Heading over Time ---
figure(2);
plot(t_uncal, headingIMUuncal)
hold on;
plot(t_cal, headingIMUcal)
grid on;
title('Heading Comparison')
legend('Uncalibrated','Calibrated')
xlabel('Time (s)')
ylabel('Heading (radians)') % Fixed label to match the data