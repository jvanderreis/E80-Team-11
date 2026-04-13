% AUV Sundial: Simplified Core Script
clear; clc; close all;

%% --- 0. SETTINGS ---
MODE_CALIBRATE = 1;         % 1 = Calibrate Offset, 0 = Estimate Time
TIMEZONE_OFFSET = -7;       % PDT is -7
local_test_hr = 17;         % Local time of dataset
local_test_min = 50; 
known_mounting_offset = 0;  % Put your calibrated offset here
year = 2026; month = 4; day = 10;

% HARDWARE DIRECTION: If the red dots move opposite to the blue line, change this!
% Use 1 for Normal, -1 for Flipped
SENSOR_DIR = -1; 

%% --- 1. LOAD & CLEAN DATA ---
filenum = '007'; % Update dataset number
logreader;

time_sec = (0:length(ch0)-1)' * 0.1; 
pt_data = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15];
imu_heading_raw = mod(headingIMU, 360);

% 1a. Strip startup glitch (first 3.5s)
stable_idx = find(time_sec > 3.5, 1, 'first');
time_sec = time_sec(stable_idx:end);
pt_data = pt_data(stable_idx:end, :);
imu_heading_raw = imu_heading_raw(stable_idx:end);
lat_val = median(lat(stable_idx:end)); 
lon_val = median(lon(stable_idx:end));

% 1b. Auto-Crop (Wait for 10-degree rotation)
baseline_heading = median(imu_heading_raw(1:50));
diff_from_baseline = abs(imu_heading_raw - baseline_heading);
diff_from_baseline(diff_from_baseline > 180) = 360 - diff_from_baseline(diff_from_baseline > 180); 
start_idx = find(diff_from_baseline > 10.0, 1, 'first');
if isempty(start_idx), start_idx = 1; end 

time_sec = time_sec(start_idx:end);
pt_data = pt_data(start_idx:end, :);
imu_heading = imu_heading_raw(start_idx:end);

%% --- 2. FIND SHADOW ANGLE ---
shadow_signal = max(pt_data(:)) - pt_data; 
angles_rad = deg2rad(linspace(0, 360 - (360/16), 16));
X_comp = sum(shadow_signal .* cos(angles_rad), 2);
Y_comp = sum(shadow_signal .* sin(angles_rad), 2);
raw_shadow_angle = mod(rad2deg(atan2(Y_comp, X_comp)), 360);

% Apply Hardware Direction (CW vs CCW)
shadow_angle = mod(SENSOR_DIR * raw_shadow_angle, 360);

%% --- 3. UNIFIED MATH ---
% The Fundamental Equation: Heading = Sun_Azimuth + Shadow_Angle + Offset
fprintf('\n=== RUN SUMMARY ===\n');

if MODE_CALIBRATE == 1
    % Calculate Sun Azimuth from known time
    utc_hr = mod(local_test_hr - TIMEZONE_OFFSET, 24);
    sun_azimuth = calculateSolarAzimuth(year, month, day, utc_hr, local_test_min, 0, lat_val, lon_val);
    
    % Solve for Offset
    calculated_offsets = imu_heading - sun_azimuth - shadow_angle;
    calculated_offsets = mod(calculated_offsets + 180, 360) - 180; % Keep between -180 and 180
    final_offset = mean(calculated_offsets);
    
    sundial_heading = mod(sun_azimuth + shadow_angle + final_offset, 360);
    fprintf('Calculated Hardware Offset: %.2f degrees\n', final_offset);
    
else
    % Solve for Sun Azimuth to guess time
    target_azimuths = mod(imu_heading - shadow_angle - known_mounting_offset, 360);
    avg_target_azimuth = mean(target_azimuths);
    
    best_err = 999; best_utc_h = 0; best_m = 0;
    for h = 0:23
        for m = 0:59
            test_az = calculateSolarAzimuth(year, month, day, h, m, 0, lat_val, lon_val);
            err = abs(test_az - avg_target_azimuth);
            if err < best_err && test_az > 60 && test_az < 300 % Daytime check
                best_err = err; best_utc_h = h; best_m = m;
            end
        end
    end
    
    local_hr = mod(best_utc_h + TIMEZONE_OFFSET, 24);
    best_az = calculateSolarAzimuth(year, month, day, best_utc_h, best_m, 0, lat_val, lon_val);
    sundial_heading = mod(best_az + shadow_angle + known_mounting_offset, 360);
    fprintf('Estimated Time: %02d:%02d Local (Error: %.2f deg)\n', local_hr, best_m, best_err);
end
fprintf('===================\n\n');

%% --- 4. PLOT ---
figure('Name', 'Simplified Sundial vs IMU', 'Position', [100, 100, 700, 400]);
plot(time_sec, imu_heading, 'b-', 'LineWidth', 1.5); hold on;
plot(time_sec, sundial_heading, 'r.', 'MarkerSize', 8);
title('Sundial vs IMU Heading'); xlabel('Time (s)'); ylabel('Heading (deg)');
legend('IMU', 'Sundial'); grid on; ylim([0 360]); yticks(0:45:360);

%% --- HELPER: SOLAR AZIMUTH ---
function azimuth = calculateSolarAzimuth(yr, mo, da, hr, mi, se, lat, lon)
    day_of_year = datenum(yr, mo, da) - datenum(yr, 1, 0);
    g = (2 * pi / 365) * (day_of_year - 1 + (hr - 12) / 24);
    eq_time = 229.18 * (0.000075 + 0.001868 * cos(g) - 0.032077 * sin(g) - 0.014615 * cos(2*g) - 0.040849 * sin(2*g));
    decl = rad2deg(0.006918 - 0.399912 * cos(g) + 0.070257 * sin(g) - 0.006758 * cos(2*g) + 0.000907 * sin(2*g) - 0.002697 * cos(3*g) + 0.00148 * sin(3*g));
    time_offset = eq_time + (4 * lon);
    tst = (hr * 60) + mi + (se / 60) + time_offset;
    ha = (tst / 4) - 180;
    lat_rad = deg2rad(lat); decl_rad = deg2rad(decl); ha_rad = deg2rad(ha);
    zenith_rad = acos(sin(lat_rad)*sin(decl_rad) + cos(lat_rad)*cos(decl_rad)*cos(ha_rad));
    az_rad = acos(-(sin(lat_rad)*cos(zenith_rad) - sin(decl_rad)) / (cos(lat_rad)*sin(zenith_rad)));
    azimuth = rad2deg(az_rad);
    if ha > 0, azimuth = mod(azimuth + 180, 360); end
end