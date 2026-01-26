% Script 2: Obstacle Course Analysis
% Author: Jack Van der Reis
% Analyzes motion data in tank, denotes acceleration peaks, and compares to
% theoretical predictions

clear; clc; close all;

% FILL IN WHEN CALCULATED/TESTED
filenum = '004'; % Change to log file from most successful obstacle course attempt
RobotMass = 2.5; % Change when weighed
logreader;

% Calibration values from Lab1Calibration.m, modify once calculated

Scale_Factor = 0.002; % Replace with your calculated Teensy Scale Factor
BiasX = 0.0000; % Replace with your ZeroX Mean
BiasY = 0.0000; % Replace with your ZeroY Mean
BiasZ = 0.0000; % Replace with your ZeroZ Mean

AccelXReal = (accelX * Scale_Factor) - BiasX;
AccelYReal = (accelY * Scale_Factor) - BiasY;
AccelZReal = (accelZ * Scale_Factor) - BiasZ;

CropStart = 1; % Run code once then look at graph and change to when robot starts schmoving
CropEnd = length(accelX); % Same thing just for the end, replace value for both

Samples = (CropStart:CropEnd)';
RunX = AccelXReal(CropStart:CropEnd);
RunY = AccelYReal(CropStart:CropEnd);
RunZ = AccelZReal(CropStart:CropEnd);

figure(1); clf;

% Calculate Maximum Thrust
MAXPWM = 200; 
DutyCycle = MAXPWM / 255;
ThrustPerMotor_N = abs(-0.6971 * DutyCycle + 0.0593); % Utilize equation from lab manual
NumMotors = 2;

% Calculate theoretical peak acceleration
TheoreticalPeak = (NumMotors * ThrustPerMotor_N) / RobotMass;

% Compare peak values of X and Y
MeasuredPeakX = max(abs(RunX));
MeasuredPeakY = max(abs(RunY));
MeasuredPeak = max(MeasuredPeakX, MeasuredPeakY);

if MeasuredPeakX >= MeasuredPeakY
    PrimaryAxis = 'X';
else
    PrimaryAxis = 'Y';
end

% X Plot
subplot(3,1,1);
plot(Samples, RunX, 'LineWidth', 1.2);
ylabel('X Acceleration (m/s^2)');
xlabel('Sample Number')
title('X Acceleration versus Sample Number');
xlim([CropStart CropEnd]);
grid on; hold on;

if PrimaryAxis == 'X'
    y_limits = ylim;
    yline(TheoreticalPeak, '--r', 'Theoretical Peak', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    text(CropStart + (CropEnd-CropStart)/2, y_limits(2)*0.8, ...
        sprintf('Measured Peak: %.3f m/s^2', MeasuredPeak), ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
end

% Y Plot
subplot(3,1,2);
plot(Samples, RunY, 'LineWidth', 1.2);
ylabel('Y Acceleration (m/s^2)');
xlabel('Sample Number')
title('Y Acceleration versus Sample Number');
xlim([CropStart CropEnd]);
grid on; hold on;

if PrimaryAxis == 'Y'
    y_limits = ylim;
    yline(TheoreticalPeak, '--r', 'Theoretical Peak', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    text(CropStart + (CropEnd-CropStart)/2, y_limits(2)*0.8, ...
        sprintf('Measured Peak: %.3f m/s^2', MeasuredPeak), ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
end

% Z Plot
subplot(3,1,3);
plot(Samples, RunZ, 'LineWidth', 1.2);
ylabel('Z Acceleration (m/s^2)');
xlabel('Sample Number');
title('Z Acceleration versus Sample Number');
xlim([CropStart CropEnd]);
grid on; hold on;

%Overall Results
fprintf('Theoretical Peak: %.4f m/s^2 (from F_net = ma)\n', TheoreticalPeak);
fprintf('Measured Peak: %.4f m/s^2 (on %s axis)\n', MeasuredPeak, PrimaryAxis);
fprintf('Difference: %.4f m/s^2\n', abs(TheoreticalPeak - MeasuredPeak));