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

Scale_Factor = 0.00000; % Replace with your calculated Teensy Scale Factor
BiasX = 0.0000; % Replace with your ZeroX Mean
BiasY = 0.0000; % Replace with your ZeroY Mean
BiasZ = 0.0000; % Replace with your ZeroZ Mean

AccelXReal = (accelX * Scale_Factor) - BiasX;
AccelYReal = (accelY * Scale_Factor) - BiasY;
AccelZReal = (accelZ * Scale_Factor) - BiasZ;

CropStart = 0; % Run code once then look at graph and change to when robot starts schmoving
CropEnd = length(accelX); % Same thing just for the end, replace value for both

Samples = (CropStart:CropEnd)';
RunX = AccelXReal(CropStart:CropEnd);
RunY = AccelYReal(CropStart:CropEnd);
RunZ = AccelZReal(CropStart:CropEnd);

% X Plot
subplot(3,1,1);
plot(Samples, RunX);
ylabel('X Acceleration (m/s^2)');
xlabel('Sample Number')
title('X Acceleration versus Sample Number');
xlim([CropStart CropEnd]);
grid on; hold on;

% Y Plot
subplot(3,1,2);
plot(Samples, RunY, 'LineWidth', 1.2);
ylabel('Y Acceleration (m/s^2)');
xlabel('Sample Number')
title('Y Acceleration versus Sample Number');
xlim([CropStart CropEnd]);
grid on; hold on;

% Z Plot
subplot(3,1,3);
plot(Samples, RunZ, 'LineWidth', 1.2);
ylabel('Z Acceleration (m/s^2)');
xlabel('Sample Number');
title('Z Acceleration versus Sample Number');
xlim([CropStart CropEnd]);
grid on; hold on;

% Theoretical Maximum Acceleration Calculations
PWM_Value = 200; % From E80_Lab_01.ino (S2: Forward section)
DutyCycle = PWM_Value / 255; 
ThrustPerMotor_N = abs(-0.6971 * DutyCycle + 0.0593);
NumMotors = 2; % We use 2 motors for the forward segment