%% ========================================================================
% E80 FINAL REPORT: MASTER DATA ASSEMBLER
% ========================================================================
clear; clc; close all;
r_earth = 6371000; 

%% --- 1. EXTRACT PHAKE LAKE DATA (LOG 018) ---
disp('Loading [Phake Lake] Data...');
filenum = '018'; 
logreader; 

t_18 = (1:length(lat))' * 0.1; 
valid_18 = (lat ~= 0) & (lon ~= 0) & ~isnan(lat) & ~isnan(lon);
lat_18 = lat(valid_18); lon_18 = lon(valid_18); 
imu_18 = headingIMU;

ch_mat = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15];
shadow_vals = max(ch_mat, [], 2) - ch_mat;
angles_rad = (0:15) * (pi / 8); 
shadow_angle_deg = mod(360 - (atan2(sum(shadow_vals .* sin(angles_rad), 2), sum(shadow_vals .* cos(angles_rad), 2)) * (180/pi)), 360);

shadow_angle_clean = unwrap(shadow_angle_deg * (pi / 180)) * (180 / pi);
shadow_angle_clean = shadow_angle_clean - (360 * round((shadow_angle_clean(1) - shadow_angle_deg(1)) / 360));
shadow_18 = movmedian(shadow_angle_clean, 5);
sun_bearing_18 = mod(imu_18 + shadow_18, 360);

%% --- 2. EXTRACT DANA POINT DATA (LOG 005) ---
disp('Loading [Dana Point] Data...');
filenum = '005'; 
logreader; 

t_05 = (1:length(lat))' * 0.1; 
valid_05 = (lat ~= 0) & (lon ~= 0) & ~isnan(lat) & ~isnan(lon);
lat_05 = lat(valid_05); lon_05 = lon(valid_05); 

origin_lat = lat_05(1); origin_lon = lon_05(1);
x_05 = (lon_05 - origin_lon) * (pi/180) * r_earth .* cos(origin_lat * pi/180);
y_05 = (lat_05 - origin_lat) * (pi/180) * r_earth;

motA_05 = motorA; 
air_raw_05 = therm_air; 
air_filt_05 = smoothdata(therm_air, 'gaussian', 50);
sun_mat_05 = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15]';

%% --- 3. BUILD THE MASTER DASHBOARD ---
disp('Assembling Final Report Figures...');
figure('Name', 'E80 Final Report Figures', 'Units', 'normalized', 'Position', [0.05, 0.05, 0.9, 0.9]);
t_layout = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- TILE 1: Phake Lake Map ---
nexttile;
geoplot(lat_18, lon_18, 'b-', 'LineWidth', 2); hold on;
geoscatter(lat_18(1), lon_18(1), 150, 'g', 'filled', 'p'); % Start Marker
geoscatter(lat_18(end), lon_18(end), 100, 'r', 'filled', 's'); % End Marker
title('[PHAKE LAKE] Global Trajectory', 'FontSize', 12);
geobasemap('satellite');

% --- TILE 2: Phake Lake Tracking ---
nexttile;
plot(t_18, imu_18, 'b', 'LineWidth', 1.5); hold on;
plot(t_18, shadow_18, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
plot(t_18, sun_bearing_18, 'g--', 'LineWidth', 2);
% Phase Markers
xline(65, 'k:', 'LineWidth', 1.5); text(10, 500, 'Transit Phase', 'FontSize', 10, 'FontWeight', 'bold');
xline(170, 'k:', 'LineWidth', 1.5); text(90, 500, 'Solar Homing Phase', 'FontSize', 10, 'FontWeight', 'bold');
text(180, 500, 'IMU Hold Phase', 'FontSize', 10, 'FontWeight', 'bold');
title('[PHAKE LAKE] Control System Logic Validation', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('IMU Heading (Body Frame)', 'Sundial Angle (Relative)', 'Calculated Sun Bearing (Global)', 'Location', 'southwest'); grid on;

% --- TILE 3: Dana Point Map ---
nexttile;
geoplot(lat_05, lon_05, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2); hold on;
geoscatter(lat_05(1), lon_05(1), 150, 'g', 'filled', 'p'); % Start Marker
geoscatter(lat_05(end), lon_05(end), 100, 'r', 'filled', 's'); % End Marker
title('[DANA POINT] Global Trajectory', 'FontSize', 12);
geobasemap('satellite');

% --- TILE 4: Dana Point Cartesian ---
nexttile;
plot(x_05, y_05, 'b-', 'LineWidth', 2); hold on;
plot([0, 15], [0, 0], 'k--', 'LineWidth', 2); % Target Egress Line
plot(x_05(1), y_05(1), 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g'); % Start
plot(x_05(end), y_05(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r'); % End
title('[DANA POINT] Local Cartesian Egress Path', 'FontSize', 12);
xlabel('X Position (m)'); ylabel('Y Position (m)');
legend('Actual Trajectory', 'Target Egress Vector (15m)', 'Location', 'best');
axis equal; grid on;

% --- TILE 5: Dana Point EMI (Dual Axis) ---
nexttile;
yyaxis left;
plot(t_05, air_raw_05, 'Color', [1 0.7 0.7], 'LineWidth', 1); hold on;
plot(t_05, air_filt_05, 'r', 'LineWidth', 2);
ylabel('Air Temp (°C)'); ylim([min(air_raw_05)-1, max(air_raw_05)+1]);

yyaxis right;
plot(t_05, motA_05, 'b-', 'LineWidth', 1); 
ylabel('Actuator Output (PWM)'); ylim([-150, 150]); 

% Phase Markers
xline(20, 'k--', 'LineWidth', 1.5); text(2, -100, 'Transit', 'FontSize', 10, 'FontWeight', 'bold');
xline(40, 'k--', 'LineWidth', 1.5); text(22, -100, 'Spin Cal.', 'FontSize', 10, 'FontWeight', 'bold');
xline(60, 'k--', 'LineWidth', 1.5); text(42, -100, 'Observation', 'FontSize', 10, 'FontWeight', 'bold');
text(70, -100, 'Solar Homing', 'FontSize', 10, 'FontWeight', 'bold');
title('[DANA POINT] Actuator EMI and Signal Conditioning', 'FontSize', 12);
xlabel('Time (s)');
legend('Raw Air Temp', 'Gaussian Filtered', 'Motor Power', 'Location', 'northwest'); grid on;

% --- TILE 6: Dana Point Heatmap ---
nexttile;
imagesc(t_05, 0:15, sun_mat_05);
cbar_heat = colorbar; cbar_heat.Label.String = 'Voltage (V)';
% Phase Markers
hold on;
xline(20, 'w--', 'LineWidth', 2); text(2, 14, 'Transit', 'Color', 'w', 'FontWeight', 'bold');
xline(40, 'w--', 'LineWidth', 2); text(22, 14, 'Spin Cal.', 'Color', 'w', 'FontWeight', 'bold');
xline(60, 'w--', 'LineWidth', 2); text(42, 14, 'Observation', 'Color', 'w', 'FontWeight', 'bold');
text(70, 14, 'Solar Homing Phase', 'Color', 'w', 'FontWeight', 'bold');
title('[DANA POINT] Optical Array Signal-to-Noise Diagnostic', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Phototransistor Channel');
set(gca, 'YDir', 'normal');

disp('Dashboard complete! Plots updated with connected paths and clinical terminology.');