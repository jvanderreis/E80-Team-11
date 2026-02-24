

% Analyzing microphone data with the FFT 
% Load the data. DAta is voltage or power, probs voltage
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



%fft 

% NEW CODE THATS NOT FFT BUT ANALYSEZ FFT DATA TO GET BEACON MAGZ

%storing magnitude in P1 

fbeacs = [9000, 11000, 13000];
M = zeros(length(fbeacs), 1); %vector to store magnitude of beacon freqs

for i = 1: length(fbeacs)
    bins = abs(f - fbeacs(i)) < 50; %bandwith taken as 50 hz to right or left of beacon frequency
    M(i) = max(P1(bins));   %taking max of P1(mag fft) for values that r true (within +-50 Hz) in bins for fbeac value
end


% Plot magnitude voltage M(i) proportional to 1/ distance, will get k slope.  


% You need to be sure that your measurements capture evidence of multipath in the tank, 
% and achieving that will require careful spacing of your measurements. Figure out how closely \
% you need to space your measurements in order to observe multipath. CHOOSE
% r matrix that ensures multipath. 

%make an r vector thats gonna be  differnet distances measured, we choose
r = [.20 .23 .26 .29 .32 .35]; % in cm its 20 cm etc  


%plot three graphs of M(i) vs 1/([r matrix])





