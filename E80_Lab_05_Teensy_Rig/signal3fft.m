% Load the data
filename = 'scope_data.csv';
data = readmatrix(filename); 

% Extract time and voltage vectors, if csv file has different column order,
% swap 1 & 2 with whatever necessary.
t = data(:, 1);
v = data(:, 2);

% Calculate sample rate (fs) and number of samples (N)
N = length(v);
dt = t(2) - t(1);
fs = 1 / dt;

% Apply the Hanning Window
window = hann(N);
vwindowed = v .* window;

% Compute the FFT
result = fft(vwindowed);

% Calculate the single-sided magnitude spectrum
P2 = abs(result / N); % Normalize dataset
P1 = P2(1:floor(N/2)+1); % Chop off negative frequencies, useless for analysis
P1(2:end-1) = 2 * P1(2:end-1); % Multiply by 2 to account for positive/negative frequencies, fixes removal from previous step

% Create the frequency vector for the x-axis
f = fs * (0:(N/2)) / N;

% Plot the result
figure;
plot(f, P1, 'b-', 'LineWidth', 1.5);
title('MATLAB FFT of Signal 3 (11 kHz Square Wave with Hanning Window)');
xlabel('Frequency (Hz)');
ylabel('Magnitude (V)');

% Set the x-axis limit to 100 kHz to match the scope's 100 kHz span setting
xlim([0 100000]); 
grid on;