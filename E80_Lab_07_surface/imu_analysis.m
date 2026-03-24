% E80 Lab 07: IMU Position Analysis (0.5m Slide Test)
clear; clc;

% Load Data
filenum = '001'; % <-- Change this to your 0.5m slide test log number
logreader;       % Natively populates accelX, accelY into the workspace

% Define Time and Constants
dt = 0.099; % Loop period from TimingOffsets.h is 99 ms
time = (0:length(accelX)-1)' * dt;
g = 9.81;   % Gravity in m/s^2

% Unit Conversion & Noise Calculation
% Convert acceleration from mg to m/s^2
ax = accelX * (g / 1000); 
ay = accelY * (g / 1000);

% Find the baseline noise (standard deviation) while the board was sitting still.
% We take the first 20 samples (approx 2 seconds) before you pushed it.
sigma_ay = std(ay(1:20)); 

% Double Integration (Acceleration -> Velocity -> Position)
% Integrate X
vx = cumtrapz(time, ax);
px = cumtrapz(time, vx);

% Integrate Y
vy = cumtrapz(time, ay);
py = cumtrapz(time, vy);

% Calculate Theoretical Uncertainty Bounds for Y
% Uncertainty = +/- 0.5 * noise * t^2
upper_bound =  0.5 * sigma_ay * (time.^2);
lower_bound = -0.5 * sigma_ay * (time.^2);

% Plot 1: X vs. Y Position (The Path)
figure(1);
plot(px, py, 'b-', 'LineWidth', 2); hold on;
% Plot the "Ideal" 0.5m path (Assuming you pushed it 0.5m along the X axis)
plot([0, 0.5], [0, 0], 'k--', 'LineWidth', 2); 
grid on;
title('Calculated Position vs Ideal 0.5m Path');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
legend('Integrated IMU Path', 'Ideal 0.5m Path', 'Location', 'best');

% Plot 2: Y Position vs. Time with Uncertainty Bounds
figure(2);
plot(time, py, 'b-', 'LineWidth', 2); hold on;
plot(time, upper_bound, 'r--', 'LineWidth', 1.5);
plot(time, lower_bound, 'r--', 'LineWidth', 1.5);
grid on;
title('Y Position Drift vs. Time');
xlabel('Time (seconds)');
ylabel('Y Position (meters)');
legend('Calculated Y Position', 'Theoretical Upper Bound', 'Theoretical Lower Bound', 'Location', 'best');