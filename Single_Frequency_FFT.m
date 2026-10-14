%% FFT on Signal
% Fs = 48000 Hz, FFT window = 32768 samples
% df = Fs / N = 48000 / 32768 = 1.46 Hz/bin
% FFT window duration = N / Fs = 32768 / 48000 = 0.683 s
% Total capture = 65536 samples = 1.365 s
% Transient buffer = 65536 - 32768 = 32768 samples = 0.683 s skipped before FFT window
Fs = 48000;   % Sampling frequency in Hz
T = 1/Fs;

% Load, Clip, and Window Signal
%file path is C:\Users\bspiller\Desktop\DaisyExamples\seed\Audio_Data_Collection
Signal = readmatrix("SPH0645_6000Hz.csv");
Clipped_Signal = Signal((end-32767:end),1);  % last 32768 samples
plot(Signal);
L = length(Clipped_Signal);
Windowed_Signal = Clipped_Signal .* hann(L);

%Run FFT on Windowed Signal
Y = fft(Windowed_Signal);
P2 = abs(Y/L); % normalize amplitude by signal length
P1 = P2(1:L/2+1); % Extract positive frequencies
P1(2:end-1) = 2*P1(2:end-1); %FFT of a real-valued signal produces a symmetric, mirrored result

%define the frequency vector in HZ
f = Fs*(0:(L/2))/L;

%Plot
figure;
plot(f, P1)
title('FFT SPH0645 6000 Hz')
xlabel('Frequency (Hz)')
ylabel('Magnitude of Positive frequencies')

%Find Dominant Frequency
% Skip DC plus a few extra low bins - near-DC drift/hum can otherwise
% dominate the peak search even after the firmware's DC-blocking filter.
% so this can't accidentally mask the actual signal.
skip_bins = 3;
[Dominant_Value, Dominant_Index] = max(P1(skip_bins+1:end));
peak_idx = Dominant_Index + skip_bins;    % true index in P1
Dominant_Frequency = f(peak_idx);
display(Dominant_Frequency);

%% SNR Measurement
signal_band = peak_idx-2 : peak_idx+2;  % 5 bins, captures full Hann window main lobe
power_spectrum = P1.^2; %square

% Exclude the same near-DC guard band used for peak-finding above - that
% low-frequency drift/hum isn't part of the audio signal, so counting it
% as "noise" would understate SNR for content that's otherwise clean in
% the audible band.
guard_band = 1:skip_bins;

signal_power = sum(power_spectrum(signal_band));
total_power = sum(power_spectrum) - sum(power_spectrum(guard_band));
noise_power = total_power - signal_power;
SNR = 10 * log10(signal_power / noise_power); %SNR (dB) = 10 × log₁₀(signal power / noise power)

display(SNR);

% RMS counts at the peak - for Test 3 (sensitivity calibration), divide
% this by the pistonphone's known Pa to get sensitivity_counts_per_Pa.
Peak_Counts = sqrt(signal_power);
display(Peak_Counts);
Sensitivity_Counts_per_Pa = Peak_Counts / 10.02;
display(Sensitivity_Counts_per_Pa);

%% THD Measurement
% Sums power at integer multiples (2x, 3x, ...) of the detected
% fundamental and compares it to the fundamental's own power. Stops
% before Nyquist since harmonics above Fs/2 are aliased/meaningless.
num_harmonics = 5; % checks 2nd through 5th harmonic
harmonic_band_halfwidth = 2; % matches signal_band convention (+-2 bins)

fundamental_power = sum(power_spectrum(signal_band));
harmonic_power_total = 0;

for h = 2:num_harmonics
    harmonic_freq = h * Dominant_Frequency;
    if harmonic_freq >= Fs/2
        break; % above Nyquist, no meaningful bin to check
    end
    [~, harmonic_idx] = min(abs(f - harmonic_freq)); % nearest bin to h*f0
    band = max(1, harmonic_idx - harmonic_band_halfwidth) : ...
           min(length(P1), harmonic_idx + harmonic_band_halfwidth);
    harmonic_power_total = harmonic_power_total + sum(power_spectrum(band));
end

THD_percent = 100 * sqrt(harmonic_power_total / fundamental_power);
THD_dB = 10 * log10(harmonic_power_total / fundamental_power);

display(THD_percent);
display(THD_dB);
