% =========================================================================
% E80 pHake Lake Analysis (Log 017)
% =========================================================================
clear; clc; close all;

%% --- 1. LOAD DATA ---
filenum = '017'; 
logreader; 

%% --- 2. DATA CLEANING & PREP ---
t = (1:length(lat))' * 0.1; 
valid_gps = (lat ~= 0) & (lon ~= 0) & ~isnan(lat) & ~isnan(lon);

lat_clean = lat(valid_gps);
lon_clean = lon(valid_gps);
wind_clean = wind_vel(valid_gps);

%% --- 3. RECONSTRUCT SUNDIAL LOGIC ---
ch_mat = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ...
          ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15];
max_volts = max(ch_mat, [], 2);
shadow_vals = max_volts - ch_mat;
angles_rad = (0:15) * (pi / 8); 
X_comp = sum(shadow_vals .* cos(angles_rad), 2);
Y_comp = sum(shadow_vals .* sin(angles_rad), 2);
shadow_angle_rad = atan2(Y_comp, X_comp);
shadow_angle_deg = mod(360 - (shadow_angle_rad * (180/pi)), 360);

shadow_rad = shadow_angle_deg * (pi / 180);
shadow_rad_unwrapped = unwrap(shadow_rad); 
shadow_angle_clean = shadow_rad_unwrapped * (180 / pi);
shift_val = 360 * round((shadow_angle_clean(1) - shadow_angle_deg(1)) / 360);
shadow_angle_clean = shadow_angle_clean - shift_val;
shadow_angle_clean = movmedian(shadow_angle_clean, 5);

%% --- 4. DATA EXTRACTION (Line of Best Fit) ---
% Isolate the data after 160s when the motors stop
idx_clean = (t > 160);
phake_air_temp = mean(therm_air(idx_clean), 'omitnan');
peak_wind = max(wind_vel);

fprintf('\n=== pHAKE LAKE RESULTS ===\n');
fprintf('Calculated Air Temp: %.2f °C\n', phake_air_temp);
fprintf('Peak Wind Velocity:  %.2f m/s\n', peak_wind);

%% --- 5. DATA VISUALIZATION ---
figure('Name', 'pHake Lake Trajectory', 'Position', [50, 50, 600, 400]);
geoplot(lat_clean, lon_clean, 'b-', 'LineWidth', 2); hold on;
geoscatter(lat_clean(1), lon_clean(1), 150, 'g', 'filled', 'p'); 
title(['pHake Lake Trajectory (Log ', filenum, ')'], 'FontSize', 12);
geobasemap('satellite'); 

figure('Name', 'pHake Lake Environmental', 'Position', [100, 100, 600, 400]);
subplot(2,1,1);
plot(t, therm_air, 'r', 'LineWidth', 1); hold on;
plot(t, therm_water, 'b', 'LineWidth', 1);
yline(phake_air_temp, 'k--', 'LineWidth', 2); % The mathematical line of best fit
text(5, phake_air_temp + 1.5, sprintf('Calculated Mean: %.2f °C', phake_air_temp), 'FontWeight', 'bold');
title('Air vs. Water Temperature Profile', 'FontSize', 12); ylabel('Temp (°C)');
legend('Raw Air Temp', 'Water Temp', 'Air Temp Mean (Motors Off)', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(t, wind_vel, 'k', 'LineWidth', 1.5);
title('Anemometer Wind Velocity', 'FontSize', 12); xlabel('Time (s)'); ylabel('Wind Speed (m/s)');
grid on;

figure('Name', 'pHake Lake Sun Tracking', 'Position', [150, 150, 600, 400]);
plot(t, headingIMU, 'b', 'LineWidth', 1.5); hold on;
plot(t, shadow_angle_clean, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
sun_bearing = mod(headingIMU + shadow_angle_clean, 360);
plot(t, sun_bearing, 'g--', 'LineWidth', 2);
title('Solar Homing Validation', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Angle (Degrees)');
legend('IMU Heading', 'Local Shadow Angle', 'Calculated Sun Bearing', 'Location', 'best');
grid on;