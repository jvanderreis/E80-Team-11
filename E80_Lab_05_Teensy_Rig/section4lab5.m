% BEACON 1
frequencyb1 = ?; % Beacon 1 Frequency in Hz
distb1 = [0.15, 0.18, 0.21, 0.24, 0.27, 0.30]; % Distances in meters (3 cm spacing)
voltageb1 = [0, 0, 0, 0, 0, 0]; % FFT Peak Voltage (Fill in your data here)

% Calculate the Analytical Model (V = k/d)
constantb1 = voltageb1(1) * distb1(1); % Calculate k using the first data point
dist_smoothb1 = linspace(min(distb1), max(distb1), 100); 
voltage_modelb1 = constantb1 ./ dist_smoothb1;

% Plot Beacon 1
figure; hold on;
plot(dist_smoothb1, voltage_modelb1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Analytical Model (V=k/d)');
scatter(distb1, voltageb1, 50, 'b', 'filled', 'DisplayName', 'Measured Data (Multipath)');
title(sprintf('Received Voltage vs. Distance (%d Hz Beacon)', frequencyb1));
xlabel('Distance from Beacon (meters)'); ylabel('Voltage Magnitude (V)');
legend('show'); grid on; hold off;

% BEACON 2
frequencyb2 = ?; % Update after identified
distb2 = [0.15, 0.18, 0.21, 0.24, 0.27, 0.30]; 
voltageb2 = [0, 0, 0, 0, 0, 0]; 

constantb2 = voltageb2(1) * distb2(1);
dist_smoothb2 = linspace(min(distb2), max(distb2), 100);
voltage_modelb2 = constantb2 ./ dist_smoothb2;

figure; hold on;
plot(dist_smoothb2, voltage_modelb2, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Analytical Model (V=k/d)');
scatter(distb2, voltageb2, 50, 'b', 'filled', 'DisplayName', 'Measured Data (Multipath)');
title(sprintf('Received Voltage vs. Distance (%d Hz Beacon)', frequencyb2));
xlabel('Distance from Beacon (meters)'); ylabel('Voltage Magnitude (V)');
legend('show'); grid on; hold off;

% BEACON 3
frequencyb3 = ?; % Update after identified
distb3 = [0.15, 0.18, 0.21, 0.24, 0.27, 0.30]; 
voltageb3 = [0, 0, 0, 0, 0, 0]; 

constantb3 = voltageb3(1) * distb3(1);
dist_smoothb3 = linspace(min(distb3), max(distb3), 100);
voltage_modelb3 = constantb3 ./ dist_smoothb3;

figure; hold on;
plot(dist_smoothb3, voltage_modelb3, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Analytical Model (V=k/d)');
scatter(distb3, voltageb3, 50, 'b', 'filled', 'DisplayName', 'Measured Data (Multipath)');
title(sprintf('Received Voltage vs. Distance (%d Hz Beacon)', frequencyb3));
xlabel('Distance from Beacon (meters)'); ylabel('Voltage Magnitude (V)');
legend('show'); grid on; hold off;