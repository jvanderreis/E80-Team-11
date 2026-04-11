% Protoboard Checkoff Graphing
% Plots Anemometer Data, Sundial Data, and Thermistor Data
clear; clc; close all;

% Run logreader to parse the binary SD card data into the workspace
filenum = '004'; 
logreader;

% Convert time from milliseconds to seconds for easier reading
if exist('time', 'var') && max(time) > 10000
    time = double(time) / 1000; 
end

%% 1. Plots Sundial Data: All 16 Channels
% Group all 16 channels into a single matrix for easy plotting
pt_data = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ...
           ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15];

figure('Name', 'Sundial: 16 Channels', 'Position', [100, 100, 800, 400]);
hold on;
for i = 1:16
    plot(time, pt_data(:, i), 'DisplayName', sprintf('Ch %d', i-1), 'LineWidth', 1.2);
end
xlabel('Time (seconds)');
ylabel('Phototransistor Voltage (V)');
title('Sundial: 16 Channels (Moving Phone in Circle)');
legend('Location', 'eastoutside');
grid on;

%% 2. Plots Sundial Data: Estimate Azimuth of "Sun"
% Assuming Phototransistor 0 is North (0 deg), Ch 1 is 22.5 deg, etc.
angles = linspace(0, 360 - (360/16), 16);

% Estimate the sun's location by finding the channel with the max voltage at each timestamp
[~, max_idx] = max(pt_data, [], 2);
azimuth_est = angles(max_idx)';

figure('Name', 'Sundial: Estimated Azimuth', 'Position', [150, 150, 800, 400]);
plot(time, azimuth_est, 'k.', 'MarkerSize', 15);
xlabel('Time (seconds)');
ylabel('Estimated Azimuth (Degrees)');
title('Sundial: Sun Azimuth Tracking Over Time');
yticks(0:45:360);
ylim([-10, 370]);
grid on;

%% 3. Plots Anemometer Data: Different Spinning Speeds
figure('Name', 'Anemometer: Frequency', 'Position', [200, 200, 800, 400]);
plot(time, anem_freq, 'b-', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Frequency (Hz)');
title('Anemometer: Spinning Speeds Over Time');
grid on;

%% 4. Plots Thermistor Data: Air vs. Water
figure('Name', 'Thermistors: Squeeze Test', 'Position', [250, 250, 800, 400]);
plot(time, therm_air, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Air Temp');
hold on;
plot(time, therm_water, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Water Temp');
xlabel('Time (seconds)');
ylabel('Temperature (\circC)');
title('Thermistor Response Over Time');
legend('Location', 'best');
grid on;