clear; clc; close all;

%% --- 1. LOAD DATA ---
filenum = '006'; % Update to your new log number!
logreader; 

% Create a relative time vector
t = (1:length(mission_state))' * 0.1; 

% Filter valid GPS data
valid_gps = (lat ~= 0) & (lon ~= 0) & ~isnan(lat) & ~isnan(lon);

%% --- 2. THE DASHBOARDS ---

% =========================================================================
% DASHBOARD 1: Navigation & GPS Trajectory
% =========================================================================
figure('Name', 'Dashboard 1: Navigation', 'Position', [50, 50, 900, 450]);

% Subplot A: Global Map
subplot(1,2,1);
geoscatter(lat(valid_gps), lon(valid_gps), 35, mission_state(valid_gps), 'filled');
colormap(lines(5)); cb = colorbar; cb.Ticks = 1:5;
title(['ASV Trajectory Colored by State (Log ', filenum, ')']);
geobasemap('satellite'); 

% Subplot B: Local XY Grid
subplot(1,2,2);
scatter(x(valid_gps), y(valid_gps), 20, mission_state(valid_gps), 'filled'); hold on;
plot(0, 0, 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'g'); 
plot(15, 0, 'rx', 'MarkerSize', 15, 'LineWidth', 3);        
plot(2, 0, 'ms', 'MarkerSize', 15, 'LineWidth', 3);         
title('Local XY Grid (Where the robot thought it was)');
xlabel('X Position (Forward)'); ylabel('Y Position (Left)');
axis equal; grid on;

% =========================================================================
% DASHBOARD 2: GNC & Motors
% =========================================================================
figure('Name', 'Dashboard 2: GNC & Motors', 'Position', [100, 100, 800, 600]);

% Subplot A: Motors
subplot(2,1,1);
plot(t, motorA, 'r', 'LineWidth', 1.5); hold on;
plot(t, motorB, 'b', 'LineWidth', 1.5);
yline(0, 'k-', 'LineWidth', 1);
title('Motor Outputs'); ylabel('PWM (-127 to 127)');
legend('Motor A (Left)', 'Motor B (Right)', 'Location', 'best'); grid on;

% Subplot B: Orientation
subplot(2,1,2);
plot(t, shadow_calc, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5); hold on;
plot(t, headingIMU, 'b', 'LineWidth', 1.5);
yline(180, 'k--', 'LineWidth', 2); 
title('Orientation Tracking'); ylabel('Angle (Degrees)'); xlabel('Time (Seconds)');
legend('Sundial Calculation', 'IMU Absolute Heading', 'Target Shadow (180 deg)', 'Location', 'best');
grid on;

% =========================================================================
% DASHBOARD 3: Optical Sundial Diagnostics (THE HEATMAP)
% =========================================================================
figure('Name', 'Dashboard 3: Optical Heatmap', 'Position', [150, 150, 800, 400]);

% Combine all 16 channels into a matrix
% Rows = Sensors (0 to 15), Cols = Time
sun_matrix = [ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ...
              ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15]';

% Plot as a heatmap
imagesc(t, 0:15, sun_matrix);
colorbar;
title('Raw Optical Sensor Voltages (Where is the Sun?)');
xlabel('Time (Seconds)');
ylabel('Phototransistor Channel (0 to 15)');
set(gca, 'YDir', 'normal'); % Keep channel 0 at the bottom

% Overlay the State boundaries for reference
hold on;
xline(20, 'w--', 'LineWidth', 2); text(5, 14, 'Transit', 'Color', 'w');
xline(40, 'w--', 'LineWidth', 2); text(25, 14, 'Spin', 'Color', 'w');
xline(60, 'w--', 'LineWidth', 2); text(45, 14, 'Pause', 'Color', 'w');
xline(100, 'w--', 'LineWidth', 2); text(75, 14, 'Sun Dash', 'Color', 'w');

%% --- 3. DATA EXTRACTION (Fixes Applied) ---
clc; disp('==================================================');
disp('          DEPLOYMENT DATA EXTRACTION              ');
disp('==================================================');

idx_pause = (mission_state == 3);
fprintf('Clean Water Temp: %.2f °C\n', mean(therm_water(idx_pause), 'omitnan'));
fprintf('Clean Air Temp:   %.2f °C\n\n', mean(therm_air(idx_pause), 'omitnan'));

idx_dash = (mission_state == 4);
steady_state = headingIMU(idx_dash);
sun_azimuth = median(steady_state(round(end/2):end), 'omitnan');
sun_azimuth = mod(sun_azimuth, 360); % Fixed the negative wrap issue!

fprintf('Estimated Sun Magnetic Azimuth: %.1f Degrees\n\n', sun_azimuth);

estimated_hour = 12 + ((sun_azimuth - 180) / 15);
hrs = floor(estimated_hour); mins = round((estimated_hour - hrs) * 60);
if mins < 0, hrs = hrs - 1; mins = 60 + mins; end
am_pm = 'AM';
if hrs >= 12, am_pm = 'PM'; if hrs > 12, hrs = hrs - 12; end; end
if hrs == 0, hrs = 12; end

fprintf('Sensor Fusion Time Estimate:    %02d:%02d %s\n', hrs, mins, am_pm);
disp('==================================================');