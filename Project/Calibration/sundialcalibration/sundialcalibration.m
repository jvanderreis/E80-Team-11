% AUV Field Calibration & Heading Analysis
% Calculates sensor gains, shadow azimuth, and compares to IMU heading
clear; clc; close all;

%% --- 1. LOAD DATA ---
filenum = '006'; % CHANGE THIS TO YOUR SD CARD FILE NUMBER
logreader;

% Generate Time Vector (10Hz sampling = 0.1s per data point)
time_sec = (0:length(ch0)-1)' * 0.1; 

% Group Sundial Channels
pt_data = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ...
           ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15];

%% --- 2. DATA TRIMMING (Remove the waiting portion) ---
% Look at Figure 1. Find the time where you actually started spinning,
% and type that number (in seconds) here!
crop_start_time = 0;   % <--- Change this to crop the beginning!
crop_end_time = max(time_sec); % <--- Change this to crop the end!

% Find indices for the crop and apply them
valid_idx = (time_sec >= crop_start_time) & (time_sec <= crop_end_time);

time_sec = time_sec(valid_idx);
pt_data = pt_data(valid_idx, :);
lat = lat(valid_idx);
lon = lon(valid_idx);

% --- BYPASS E80 LIBRARY: CALCULATE IMU HEADING MANUALLY ---
% Use the raw Magnetometer X and Y vectors to find magnetic north
imu_heading_rad = atan2(magY(valid_idx), magX(valid_idx));

% Convert from radians to degrees
imu_heading = rad2deg(imu_heading_rad);

% Shift the scale so it's 0 to 360 (instead of -180 to 180)
imu_heading = mod(imu_heading, 360);

%% --- 3. CALCULATE PHOTOTRANSISTOR GAINS ---
calib_max = max(pt_data); 
calib_min = min(pt_data); 
avg_peak = mean(calib_max);
sensor_gains = avg_peak ./ calib_max;

fprintf('\n--- CALIBRATION GAINS ---\n');
fprintf('float sensor_gains[16] = {');
fprintf('%.2f, ', sensor_gains(1:15));
fprintf('%.2f};\n', sensor_gains(16));
fprintf('---------------------------\n\n');

%% --- 4. SHADOW INVERSION & CIRCULAR CENTROID ---
pt_calibrated = (pt_data - calib_min) .* sensor_gains;
shadow_signal = max(pt_calibrated, [], 2) - pt_calibrated;

angles_rad = deg2rad(linspace(0, 360 - (360/16), 16));
X_components = sum(shadow_signal .* cos(angles_rad), 2);
Y_components = sum(shadow_signal .* sin(angles_rad), 2);

shadow_relative_angle = rad2deg(atan2(Y_components, X_components));
shadow_relative_angle(shadow_relative_angle < 0) = shadow_relative_angle(shadow_relative_angle < 0) + 360;
sun_relative_angle = mod(shadow_relative_angle + 180, 360);

%% --- 5. CALCULATE TRUE SOLAR AZIMUTH FROM GPS ---
test_lat = median(lat);
test_lon = median(lon);

% TIME OVERRIDE: Set to the time you did the test! (e.g., 22:10 UTC = 3:10 PM PDT)
test_hr = 22;   
test_min = 10;  
test_sec = 0;
year = 2026; month = 4; day = 9;

true_solar_azimuth = calculateSolarAzimuth(year, month, day, test_hr, test_min, test_sec, test_lat, test_lon);
fprintf('Calculated True Solar Azimuth: %.2f degrees\n', true_solar_azimuth);

%% --- 6. CALCULATE TRUE ROBOT HEADING ---
% We use 'sun_relative_angle' from Section 4 (Circular Centroid).
% We add 180 here to account for the mounting offset seen in your data.
sundial_true_heading = mod(true_solar_azimuth - sun_relative_angle + 180, 360);

%% --- 7. PLOTTING: ALL THE INTERESTING FIGURES ---

% Figure 1: Raw Voltages
figure('Name', 'Raw Sensor Voltages', 'Position', [50, 50, 800, 400]);
plot(time_sec, pt_data);
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Raw Sundial Sensor Voltages (Use this to adjust crop\_start\_time!)');
grid on;

% Figure 2: Shadow Waterfall (Heatmap)
figure('Name', 'Shadow Heatmap', 'Position', [100, 100, 800, 400]);
imagesc('XData', time_sec, 'YData', 0:15, 'CData', pt_data');
colorbar;
xlabel('Time (s)'); ylabel('Sensor Channel (0-15)');
title('Sundial "Waterfall" (Dark band is the shadow moving as you spin)');
set(gca, 'YDir', 'normal');

% Figure 3: Heading Comparison
figure('Name', 'Heading Comparison', 'Position', [150, 150, 800, 400]);
plot(time_sec, sundial_true_heading, 'r.', 'MarkerSize', 8, 'DisplayName', 'Sundial Optical Heading');
hold on;
plot(time_sec, imu_heading, 'b-', 'LineWidth', 1.5, 'DisplayName', 'IMU Magnetometer Heading');
xlabel('Time (s)'); ylabel('True Heading (Degrees)');
title('Field Test: Sundial Compass vs. Magnetometer Compass');
legend('Location', 'best');
yticks(0:45:360); ylim([-10, 370]);
grid on;

% Figure 4: Heading Error
% Calculate difference and handle the 360-degree wrap around
heading_error = sundial_true_heading - imu_heading;
heading_error = mod(heading_error + 180, 360) - 180; 

figure('Name', 'Heading Error', 'Position', [200, 200, 800, 400]);
plot(time_sec, heading_error, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (Degrees)');
title('Heading Error (Sundial - IMU)');
ylim([-180, 180]); yticks(-180:45:180);
grid on;

%% --- HELPER FUNCTION: ASTRONOMICAL SOLAR AZIMUTH ---
function azimuth = calculateSolarAzimuth(yr, mo, da, hr, mi, se, lat, lon)
    day_of_year = datenum(yr, mo, da) - datenum(yr, 1, 0);
    g = (2 * pi / 365) * (day_of_year - 1 + (hr - 12) / 24);
    
    eq_time = 229.18 * (0.000075 + 0.001868 * cos(g) - 0.032077 * sin(g) ...
              - 0.014615 * cos(2*g) - 0.040849 * sin(2*g));
          
    decl = 0.006918 - 0.399912 * cos(g) + 0.070257 * sin(g) ...
           - 0.006758 * cos(2*g) + 0.000907 * sin(2*g) ...
           - 0.002697 * cos(3*g) + 0.00148 * sin(3*g);
    decl = rad2deg(decl);
    
    time_offset = eq_time + (4 * lon);
    tst = (hr * 60) + mi + (se / 60) + time_offset;
    ha = (tst / 4) - 180;
    
    lat_rad = deg2rad(lat);
    decl_rad = deg2rad(decl);
    ha_rad = deg2rad(ha);
    
    zenith_rad = acos(sin(lat_rad)*sin(decl_rad) + cos(lat_rad)*cos(decl_rad)*cos(ha_rad));
    az_rad = acos(-(sin(lat_rad)*cos(zenith_rad) - sin(decl_rad)) / (cos(lat_rad)*sin(zenith_rad)));
    
    azimuth = rad2deg(az_rad);
    if ha > 0
        azimuth = mod(azimuth + 180, 360);
    end
end