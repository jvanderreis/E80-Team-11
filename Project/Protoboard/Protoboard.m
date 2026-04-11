% Breadboard Checkoff Graphing
% Plots Anemometer Data, Sundial Data, and Thermistor Data
clear; clc; close all;

% Run logreader to parse the binary SD card data into the workspace
% MAKE SURE THIS NUMBER MATCHES THE FILE ON YOUR SD CARD!
filenum = '000'; 
logreader;

% Since the Arduino logs exactly one data point every 99ms (0.099 seconds),
% we can build the time vector by multiplying the data point index by 0.099.
time_sec = (0:length(ch0)-1)' * 0.099; 

%% 1. Plots Sundial Data: All 16 Channels
% Group all 16 channels into a single matrix for easy plotting
pt_data = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ...
           ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15];

figure('Name', 'Sundial: 16 Channels', 'Position', [100, 100, 800, 400]);
hold on;
for i = 1:16
    plot(time_sec, pt_data(:, i), 'DisplayName', sprintf('Ch %d', i-1), 'LineWidth', 1.2);
end
xlabel('Time (seconds)');
ylabel('Phototransistor Voltage (V)');
title('Sundial: 16 Channels (Moving Phone in Circle)');
legend('Location', 'eastoutside');
grid on;

%% 2. Plots Sundial Data: Continuous Azimuth (Circular Centroid)
% Generate the angles for the 16 sensors in RADIANS for the trig math
angles_deg = linspace(0, 360 - (360/16), 16);
angles_rad = deg2rad(angles_deg);

% Calibrate the baseline darkness (subtract the minimum voltage)
% so that sensors in the dark don't "pull" the math off target.
pt_data_calibrated = pt_data - min(pt_data);

% --- Calculate the Vector Sum (Circular Centroid) ---
% Multiply each voltage by the cosine (X) and sine (Y) of its physical angle
% Then sum them all up across the 16 channels for every single time step
X_components = sum(pt_data_calibrated .* cos(angles_rad), 2);
Y_components = sum(pt_data_calibrated .* sin(angles_rad), 2);

% Use atan2 to find the exact angle of the resulting vector
azimuth_est_continuous = rad2deg(atan2(Y_components, X_components));

% atan2 outputs from -180 to +180. Wrap the negative values to 0-360 degrees.
azimuth_est_continuous(azimuth_est_continuous < 0) = azimuth_est_continuous(azimuth_est_continuous < 0) + 360;

% --- Plot the Results ---
figure('Name', 'Sundial: Continuous Azimuth', 'Position', [150, 150, 800, 400]);
plot(time_sec, azimuth_est_continuous, 'k.', 'MarkerSize', 8);
xlabel('Time (seconds)');
ylabel('Estimated Azimuth (Degrees)');
title('Sundial "Sun" Tracking');
yticks(0:45:360);
ylim([-10, 370]);
grid on;

%% 3. Plots Anemometer Data: Different Spinning Speeds
figure('Name', 'Anemometer: Frequency', 'Position', [200, 200, 800, 400]);
plot(time_sec, anem_freq, 'b-', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Frequency (Hz)');
title('Anemometer: Spinning Frequencies versus Time');
grid on;

%% 4. Plots Thermistor Data: Air vs. Water
figure('Name', 'Thermistors: Squeeze Test', 'Position', [250, 250, 800, 400]);
plot(time_sec, therm_air, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Air Temp');
hold on;
plot(time_sec, therm_water, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Water Temp');
xlabel('Time (seconds)');
ylabel('Temperature (\circC)');
title('Thermistor Response Over Time');
legend('Location', 'best');
grid on;