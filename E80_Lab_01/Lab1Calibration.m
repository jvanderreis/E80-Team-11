% Script 1: Calibration & Stats
% Requirements: Flat on table Log (Number unknown) and Side Log (Number
% unknown)
clear; clc; close all;

% Load Flat File
filenum = '001'; 
logreader; 
ZeroX = accelX; ZeroY = accelY; GravityZ = accelZ;

% Load Side File
filenum = '002';
logreader;
ZeroZ = accelZ;

% Calculate Teensy Scale Factor (WRITE DOWN!)
mean_1g = mean(GravityZ);
mean_0g = mean(ZeroZ);
Scale_Factor = 9.81 / (mean_1g - mean_0g);

fprintf('The Teensy Scale Factor is %.6f', Scale_Factor);

% Z axis Testing

TrueZ = GravityZ * Scale_Factor;

Zmean = mean(TrueZ); % Z axis mean
Zstd = std(TrueZ); % Z standard deviation
Zn = length(TrueZ); % How many data points for Z
Zse = Zstd / sqrt(Zn); % Standard Error for Z
ConfidenceLower = Zmean - (1.96 * Zse); % Lower confidence bound for Z
ConfidenceUpper = Zmean + (1.96 * Zse); % Upper confidence bound for Z

fprintf('The mean value for Z acceleration is %.4f m/s^2\n', Zmean);
fprintf('The standard deviation for Z is %.4f\n', Zstd);
fprintf('The Standard Error for Z is %.4f\n', Zse);
fprintf('The 95%% CI Bounds are [%.4f, %.4f]\n', ConfidenceLower, ConfidenceUpper);

% T-Tests determine if "zero" results are statistically
% similar and therefore accurate (e.g. H0 = X and Y are similar, Ha = X and Y
% are dissimilar and something is going wrong). "h" determines if the null
% should be rejected (1 = reject, 0 = accept)

% T-Test 1: Zero X vs Zero Y
[h, p] = ttest2(ZeroX, ZeroY);
fprintf('X vs Y: P-Value is %.4e', p);
% T-Test 2: Zero X vs Zero Z (Using Side Data for Z)
[h2, p2] = ttest2(ZeroX, ZeroZ);
fprintf('X vs Z: P-Value is %.4e', p2);

% T-Test 3: Zero Y vs Zero Z (Using Side Data for Z)
[h3, p3] = ttest2(ZeroY, ZeroZ);
fprintf('Y vs Z: P-Value is %.4e', p3);

