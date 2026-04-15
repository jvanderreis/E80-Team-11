% E80 data analysis phake lake

clear; clc; close all;

%% --- 1. LOAD DATA ---
% Ensure your 'logreader' script creates the variables shown in your 'whos' output
filenum = '018'; 
logreader; 

%% --- 2. DATA CLEANING & PREP ---
% Create a sample/time vector (Assuming 10Hz logging based on a 100ms activePeriod)
t = (1:length(lat))' * 0.1; 

% Filter out invalid GPS points (before lock or dropped signals)
% Valid GPS points usually aren't exactly 0, 0
valid_gps = (lat ~= 0) & (lon ~= 0) & ~isnan(lat) & ~isnan(lon);

% Extract valid trajectory for mapping
lat_clean = lat(valid_gps);
lon_clean = lon(valid_gps);
wind_clean = wind_vel(valid_gps);

%% --- 3. RECONSTRUCT SUNDIAL LOGIC ---
% Since 'shadow_angle_deg' isn't in your workspace, we recreate your C++ logic here.
% Combine the 16 channels into an N x 16 matrix
ch_mat = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ...
          ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15];

% Find the max voltage at each time step (across the 16 sensors)
max_volts = max(ch_mat, [], 2);

% Calculate shadow intensity (max - current)
shadow_vals = max_volts - ch_mat;

% Sensor angles (0 to 15 * 22.5 degrees)
angles_rad = (0:15) * (pi / 8); 

% Calculate X and Y vector components
X_comp = sum(shadow_vals .* cos(angles_rad), 2);
Y_comp = sum(shadow_vals .* sin(angles_rad), 2);

% Calculate final angle, applying your C++ flip/mod logic
shadow_angle_rad = atan2(Y_comp, X_comp);
shadow_angle_deg = mod(360 - (shadow_angle_rad * (180/pi)), 360);

%% --- 4. DATA VISUALIZATION FOR REPORT ---

% --- FIGURE 1: ASV Trajectory Map ---
figure('Name', 'Deployment Trajectory', 'Position', [100, 100, 800, 600]);
% Plot the path using geoscatter, coloring points by Wind Velocity
geoscatter(lat_clean, lon_clean, 25, wind_clean, 'filled');
colormap(parula);
c = colorbar;
c.Label.String = 'Wind Velocity (m/s)';
title(['ASV Deployment Trajectory (Log ', filenum, ')']);
% Use a satellite base map for a highly professional report aesthetic
geobasemap('satellite'); 

% --- FIGURE 2: Environmental Conditions ---
figure('Name', 'Environmental Data', 'Position', [150, 150, 800, 600]);

% Subplot 1: Temperature Transect
subplot(2,1,1);
plot(t, therm_air, 'r', 'LineWidth', 1.5); hold on;
plot(t, therm_water, 'b', 'LineWidth', 1.5);
title('Air vs. Water Temperature Profile');
ylabel('Temperature (°C)');
legend('Air Temp', 'Water Temp', 'Location', 'best');
grid on;

% Subplot 2: Wind Speed
subplot(2,1,2);
plot(t, wind_vel, 'k', 'LineWidth', 1.5);
title('Anemometer Wind Velocity');
xlabel('Time (Seconds)');
ylabel('Wind Speed (m/s)');
grid on;

%% --- 3b. LIGHTLY CLEAN SHADOW ANGLE DATA ---

% 1. Fix the 0-360 degree wrap-around artifact
% Convert to radians, unwrap (removes artificial jumps > 180 deg), back to degrees
shadow_rad = shadow_angle_deg * (pi / 180);
shadow_rad_unwrapped = unwrap(shadow_rad); 
shadow_angle_clean = shadow_rad_unwrapped * (180 / pi);

% (Optional but recommended) Unwrapping can sometimes shift the whole dataset 
% up or down by 360 degrees. This line shifts it back so it aligns with your IMU.
shift_val = 360 * round((shadow_angle_clean(1) - shadow_angle_deg(1)) / 360);
shadow_angle_clean = shadow_angle_clean - shift_val;

% 2. Apply a light moving median filter
% A window of 5 samples will remove 1-off sensor spikes without destroying 
% the actual dynamic response/curve of the robot's movement.
shadow_angle_clean = movmedian(shadow_angle_clean, 5);

% --- FIGURE 3: Sun Seeking Performance ---
figure('Name', 'Sun Tracking Performance', 'Position', [200, 200, 800, 450]);
plot(t, headingIMU, 'b', 'LineWidth', 1.5); hold on;
plot(t, shadow_angle_clean, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
sun_bearing = mod(headingIMU + shadow_angle_clean, 360);
plot(t, sun_bearing);
title('Phase 2 Sun-Seeking & IMU Hold Validation');
xlabel('Time (Seconds)');
ylabel('Angle (Degrees)');
legend('IMU Heading (Robot Direction)', 'Local Shadow Angle (Sun Relative)', 'Location', 'best');
grid on;