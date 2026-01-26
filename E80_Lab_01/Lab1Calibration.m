% Script 1: Calibration & Stats
% Author: Jack Van der Reis
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

% Z axis Statistical Testing

TrueZ = GravityZ * Scale_Factor;

Zmean = mean(TrueZ); % Z axis mean
Zstd = std(TrueZ); % Z standard deviation
Zn = length(TrueZ); % How many data points for Z
Zse = Zstd / sqrt(Zn); % Standard Error for Z
ConfidenceLowerZ = Zmean - (1.96 * Zse); % Lower confidence bound for Z
ConfidenceUpperZ = Zmean + (1.96 * Zse); % Upper confidence bound for Z

fprintf('The mean value for Z acceleration is %.4f m/s^2\n', Zmean);
fprintf('The standard deviation for Z is %.4f\n', Zstd);
fprintf('The Standard Error for Z is %.4f\n', Zse);
fprintf('The 95%% CI Bounds are [%.4f, %.4f]\n', ConfidenceLowerZ, ConfidenceUpperZ);

% X, Y, and Z Axis Confidence Intervals when No Acceleration

TrueX = ZeroX * Scale_Factor;
TrueY = ZeroY * Scale_Factor;
TrueZ0 = ZeroZ * Scale_Factor;

Xmean = mean(TrueX);
Xstd = std(TrueX);
Xn = length(TrueX);
Xse = Xstd / sqrt(Xn);
ConfidenceLowerX = Xmean - (1.96 * Xse);
ConfidenceUpperX = Xmean + (1.96 * Xse);

Ymean = mean(TrueY);
Ystd = std(TrueY);
Yn = length(TrueY);
Yse = Ystd / sqrt(Yn);
ConfidenceLowerY = Ymean - (1.96 * Yse);
ConfidenceUpperY = Ymean + (1.96 * Yse);

Z0mean = mean(TrueZ0);
Z0std = std(TrueZ0);
Z0n = length(TrueZ0);
Z0se = Z0std / sqrt(Z0n);
ConfidenceLowerZ0 = Z0mean - (1.96 * Z0se);
ConfidenceUpperZ0 = Z0mean + (1.96 * Z0se);

fprintf('The mean for X acceleration at rest is %.4f m/s^2 with a confidence interval of [%.4f, %.4f]. This confirms the null hypothesis that the robot remains near zero acceleration on the X axis when at rest\n', Xmean, ConfidenceLowerX, ConfidenceUpperX)
fprintf('The mean for Y acceleration at rest is %.4f m/s^2 with a confidence interval of [%.4f, %.4f]. This confirms the null hypothesis that the robot remains near zero acceleration on the Y axis when at rest\n', Ymean, ConfidenceLowerY, ConfidenceUpperY)
fprintf('The mean for Z acceleration at rest is %.4f m/s^2 with a confidence interval of [%.4f, %.4f]. This confirms the null hypothesis that the robot remains near zero acceleration on the Z axis when at rest\n', Z0mean, ConfidenceLowerZ0, ConfidenceUpperZ0)

% T-Tests determine if "zero" results are statistically
% similar and therefore accurate (e.g. H0 = X and Y are similar, Ha = X and Y
% are dissimilar and something is going wrong). "h" determines if the null
% should be rejected (1 = reject, 0 = accept)

% T-Test 1: Zero X vs Zero Y
[h, p] = ttest2(ZeroX, ZeroY);
fprintf('X vs Y: P-Value is %.4e\n', p);

% T-Test 2: Zero X vs Zero Z (Using Side Data for Z)
[h2, p2] = ttest2(ZeroX, ZeroZ);
fprintf('X vs Z: P-Value is %.4e\n', p2);

% T-Test 3: Zero Y vs Zero Z (Using Side Data for Z)
[h3, p3] = ttest2(ZeroY, ZeroZ);
fprintf('Y vs Z: P-Value is %.4e\n', p3);

% Graphs for Submission

figure(1); clf;

% Subplot 1: Zero X (Top Left)
subplot(2,2,1); 
plot(ZeroX); 
title('Zero X Acceleration'); 
ylabel('Raw Teensy Units');

% Subplot 2: Zero Y (Top Right)
subplot(2,2,2); 
plot(ZeroY); 
title('Zero Y Acceleration'); 
ylabel('Raw Teensy Units');

% Subplot 3: Zero Z (Bottom Left)
subplot(2,2,3); 
plot(ZeroZ); 
title('Zero Z Acceleration'); 
ylabel('Raw Teensy Units');

% Subplot 4: Gravity Z (Bottom Right)
subplot(2,2,4); 
plot(GravityZ); 
title('Acceleration Due to Gravity Z (Flat)'); 
ylabel('Raw Teensy Units');
