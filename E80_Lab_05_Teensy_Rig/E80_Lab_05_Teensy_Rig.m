%% Lab 5 Interface

samplingFreq = 100E3; % Hz [100E3 max]
numSamples = 1000; % the higher this is the longer sampling will take

bytesPerSample = 2; % DO NOT CHANGE
micSignal = zeros(numSamples,1); % DO NOT CHANGE

% close and delete serial ports in case desired port is still open
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% Modify first argument of serial to match Teensy port under Tools tab of Arduino IDE.  Second to match baud rate.
% Note that the timeout is set to 60 to accommodate long sampling times.
s = serial('COM10','BaudRate',115200); 
set(s,{'InputBufferSize','OutputBufferSize'},{numSamples*bytesPerSample,4});
s.Timeout = 60; 

fopen(s);
pause(2);
fwrite(s,[numSamples,samplingFreq/2],'uint16');
dat = fread(s,numSamples,'uint16');
fclose(s);

% Some convenience code to begin converting data for you.
micSignal = dat.*(3.3/1023); % convert from Teensy Units to Volts
samplingPeriod = 1/samplingFreq; % s
totalTime = numSamples*samplingPeriod; % s
t = linspace(0,totalTime,numSamples)'; % time vector of signal

figure;
plot(t, micSignal, 'b-', 'LineWidth', 1);
title(sprintf('Teensy Time Domain (Sample Rate: %d Hz)', samplingFreq));
xlabel('Time (seconds)');
ylabel('Voltage (V)');
xlim([0, 0.005]); % Zooms in to see the actual waves
grid on;

% Calculate FFT
N = numSamples;
window = hann(N); % Apply window to prevent spectral leakage
micSignal_windowed = micSignal .* window;

result = fft(micSignal_windowed);
P2 = abs(result / N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2 * P1(2:end-1); % Single-sided magnitude

f = samplingFreq * (0:(N/2)) / N; % Create frequency x-axis

% Plot FFT
figure;
plot(f, P1, 'r-', 'LineWidth', 1.5);
title(sprintf('Teensy FFT Magnitude (Sample Rate: %d Hz)', samplingFreq));
xlabel('Frequency (Hz)');
ylabel('Magnitude (V)');
grid on;