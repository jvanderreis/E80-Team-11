% BEACON 1
frequencyb1 = 9000; % Beacon 1 Frequency in Hz
distb1 = [0.15, 0.18, 0.21, 0.24, 0.27, 0.30, 0.33, 0.36, 0.39, 0.42, 0.45, 0.48, 0.51, 0.54, 0.57, 0.60, 0.63, 0.66]; % Distances in meters (3 cm spacing)
voltageb1 = [6, 7, 3, 4.5, 2.5, 4, 3.5, 1.1, 1, 3, 3, 1, 2.5, 2.5, 2, 1.5, 1, 2]; % FFT Peak Voltage (Fill in your data here)
voltageb1 = 0.001.*voltageb1;

% Calculate the Analytical Model (V = k/d)
constantb1 = voltageb1(1) * distb1(1); % Calculate constant that accounts for volume, sensitivity, and gain
dist_smoothb1 = linspace(min(distb1), max(distb1), 100); 
voltage_modelb1 = constantb1 ./ dist_smoothb1;

% Plot Beacon 1
figure; hold on;
plot(dist_smoothb1, voltage_modelb1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Analytical Model (V=k/d)');
scatter(distb1, voltageb1, 50, 'b', 'filled', 'DisplayName', 'Measured Data (Multipath)');
title(sprintf('Received Voltage vs. Distance (%d Hz Beacon)', frequencyb1));
xlabel('Distance from Beacon (meters)'); ylabel('Voltage Magnitude (V)');
legend('show'); grid on; hold off;