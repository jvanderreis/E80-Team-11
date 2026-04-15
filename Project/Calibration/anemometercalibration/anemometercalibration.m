% Anemometer Wind Tunnel Calibration (Updated for Main Script)
clear; clc; close all;

% --- 1. LOAD DATA ---
filenum = '016'; % CHANGE THIS TO YOUR SD CARD FILE NUMBER
logreader;

% Use anem_freq from your WeatherSampler
time_sec = (0:length(anem_freq)-1)' * 0.1; 

% --- 2. PLOT THE "STAIRCASE" ---
figure('Name', 'Raw Wind Tunnel Data', 'Position', [100, 500, 800, 400]);
plot(time_sec, anem_freq, 'b-', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Measured Frequency (Hz)');
title('Anemometer "Staircase" (Read flat steps from here)');
grid on;

%% --- 3. MANUAL CALIBRATION DATA ENTRY ---
% 1. Type the Wind Tunnel RPMs you used into this array
fan_speed_rpm = [307, 486, 648, 817, 163, 346, 619]; % <--- REPLACE ME

% 2. Read the flat Hz levels from the graph above and put them in this array
measured_hz = [1.546, 2.577, 3.861, 5.154, 0.498, 1.718, 3.861];  % <--- REPLACE ME

% Convert RPM to True Velocity using your Wind Tunnel calibration equation
% Equation: Velocity = 0.01549 * RPM - 0.3552
true_velocity_mps = (0.01549 .* fan_speed_rpm) - 0.3552;

%% --- 4. CALCULATE ANEMOMETER CALIBRATION CURVE (Linear Fit) ---
% Fits the data to the equation: Velocity = m * Frequency + b
p = polyfit(measured_hz, true_velocity_mps, 1);
slope = p(1);
intercept = p(2);

fprintf('\n--- ANEMOMETER CALIBRATION EQUATION ---\n');
fprintf('Velocity (m/s) = (%.4f * anem_freq) + %.4f\n', slope, intercept);
fprintf('---------------------------------------\n\n');

% --- 5. PLOT THE CALIBRATION CURVE ---
figure('Name', 'Anemometer Calibration Curve', 'Position', [100, 50, 800, 400]);
plot(measured_hz, true_velocity_mps, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Test Points');
hold on;

fit_x = linspace(0, max(measured_hz)*1.1, 100);
fit_y = polyval(p, fit_x);
plot(fit_x, fit_y, 'r-', 'LineWidth', 2, 'DisplayName', 'Linear Fit');

xlabel('Anemometer Frequency (Hz)');
ylabel('True Wind Velocity (m/s)');
title('Anemometer Calibration Function');
legend('Location', 'northwest');
grid on;