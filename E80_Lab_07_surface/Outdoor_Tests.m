%% E80 Lab 07: Surface Navigation & Control Analysis
% INSTRUCTIONS:
% Put this script, logreader.m, and your HMC_Campus.png in the same folder.
% Put your SD card .BIN files in the folder.
% Change the 'filenum' variable below to match the test you are analyzing.
% Run it once for your Drunken Path test (ignore Figure 2).
% Run it again for your P-Control test (use all figures).

clear; clc;

filenum = '004'; % <-- CHANGE THIS: e.g. '001' for Loop Test, '002' for P-Control
logreader;

% --- DATA TRUNCATION ---
% Find the first index where the robot moves past y = -38.
% (Assuming it starts near 0 and moves South towards -40)
start_idx = find(y <= -38, 1, 'first');

% Safety check: If the robot never hit -38, keep the original data
if ~isempty(start_idx)
    % Slice all necessary variables from the start index to the end
    x       = x(start_idx:end);
    y       = y(start_idx:end);
    yaw     = yaw(start_idx:end);
    yaw_des = yaw_des(start_idx:end);
    u       = u(start_idx:end);
    
    % Note: If logreader.m imports other variables you need later 
    % (like raw GPS lat/lon), make sure to slice them here too!
else
    disp('Warning: Target y = -38 was never reached. Using full dataset.');
end
% -----------------------

dt = 0.099;
% Because y is now shorter, time will automatically scale correctly 
% and start at exactly t = 0 for your new data slice.
time = (0:length(y)-1)' * dt;

%% FIGURE 1: GPS Path Overlaid on Campus Map
figure(1);
img = imread('HMC_Campus.png'); 

% TUNE YOUR MAP BOUNDARIES HERE
% These numbers define the physical "box" in meters that your image covers.
% Tweak these 4 values until your starting (0,0) point sits exactly 
% outside Parsons, and your path aligns with the sidewalks.
% TUNE YOUR MAP BOUNDARIES HERE
map_West  = 65;   % Shifted right to move the mall under the data
map_East  = 265;  % 
map_South = -120; % Shifted down to move the mall under the data
map_North = 20;   %

% Stretch the image over your defined meter grid
imshow(img, 'XData', [map_West, map_East], 'YData', [map_North, map_South]);
set(gca, 'YDir', 'normal'); % Ensures North points UP
hold on;

% Plot your actual recorded GPS path in bright yellow
plot(x, y, 'y-', 'LineWidth', 2.5);

% Plot the Ideal P-Control Trajectory in dashed red
% The waypoints are: (125,-40) to (150,-40) to (125,-40).
% Note: This will show up on your Drunken Path test too, but you can just 
% ignore it or comment these two lines out for that specific run.
ideal_x = [125, 150, 125];
ideal_y = [-40, -40, -40];
plot(ideal_x, ideal_y, 'r--', 'LineWidth', 2);

title(['GPS Path Overlaid on Campus - Log ', filenum]);
xlabel('Meters East of Origin'); 
ylabel('Meters North of Origin'); 
legend('Actual Path', 'Ideal P-Control Path', 'Location', 'best');
axis on; grid on;

%% FIGURE 2: Control Effort & Angle Error (P-Control Test Only)
% Note: If you are processing your first "Drunken Path" test, this data 
% will just be noise. Only use this figure for your P-Control submission.

figure(2);

% Top Plot: Angle Error
subplot(2,1,1);
plot(time, (yaw_des - yaw), 'r', 'LineWidth', 1.5);
title('Angle Error vs. Time'); 
ylabel('Error (radians)'); 
grid on;

% Bottom Plot: Control Effort (u)
subplot(2,1,2);
plot(time, u, 'b', 'LineWidth', 1.5);
title('Control Effort (u) vs. Time'); 
xlabel('Time (seconds)'); 
ylabel('Effort'); 
grid on;