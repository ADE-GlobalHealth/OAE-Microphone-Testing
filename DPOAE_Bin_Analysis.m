%% Frequency Response Analysis
% One tone at a time, one capture file per tone. Reads only the bin
% matching each capture's own tone frequency - not all 6 DPOAE bins out
% of a single file, since a single-tone capture only has a meaningful
% signal at that one frequency; the other bins would just be noise/
% harmonics from that same test, not the mic's actual response there.
Fs = 48000;   % Sampling frequency in Hz
bin_halfwidth = 2; % +-2 bins around each target frequency

% Sensitivity constant from Test 3 (pistonphone calibration) - counts per
% Pascal for this candidate at the locked gain setting. Set this before
% running; every dB SPL value below depends on it being accurate.
sensitivity_counts_per_Pa = 0.0031;

% One row per stepped tone: {capture file, frequency actually played in
% that capture}. Update the filenames to match your actual capture files.
tone_captures = {
    "SPH0645_600Hz.csv",  600;
    "SPH0645_1200Hz.csv", 1200;
    "SPH0645_2000Hz.csv", 2000;
    "SPH0645_3000Hz.csv", 3000;
    "SPH0645_4000Hz.csv", 4000;
    "SPH0645_6000Hz.csv", 6000;
};

num_tones = size(tone_captures, 1);
results_hz = zeros(num_tones, 1);
results_dB_SPL = zeros(num_tones, 1);

fprintf('\n%-22s %-10s %-14s %-14s %-12s\n', 'File', 'Freq (Hz)', 'Level (counts)', 'Level (dBFS)', 'dB SPL');

for i = 1:num_tones
    filename = tone_captures{i, 1};
    target_hz = tone_captures{i, 2};

    % Load, Clip, and Window Signal
    Signal = readmatrix(filename);
    Clipped_Signal = Signal((end-32767:end), 1);  % last 32768 samples
    L = length(Clipped_Signal);
    Windowed_Signal = Clipped_Signal .* hann(L);

    %Run FFT on Windowed Signal
    Y = fft(Windowed_Signal);
    P2 = abs(Y/L); % normalize amplitude by signal length
    P1 = P2(1:L/2+1); % Extract positive frequencies
    P1(2:end-1) = 2*P1(2:end-1); %FFT of a real-valued signal produces a symmetric, mirrored result

    %define the frequency vector in HZ
    f = Fs*(0:(L/2))/L;
    power_spectrum = P1.^2;

    [~, bin_idx] = min(abs(f - target_hz)); % nearest bin to this capture's own tone
    band = max(1, bin_idx - bin_halfwidth) : min(length(P1), bin_idx + bin_halfwidth);
    bin_counts = sqrt(sum(power_spectrum(band))); % RMS amplitude in the band
    bin_dBFS = 20 * log10(bin_counts);

    pascals = bin_counts / sensitivity_counts_per_Pa;
    dB_SPL = 20 * log10(pascals / 20e-6);

    results_hz(i) = target_hz;
    results_dB_SPL(i) = dB_SPL;

    fprintf('%-22s %-10g %-14g %-14.2f %-12.2f\n', filename, target_hz, bin_counts, bin_dBFS, dB_SPL);
end

%% Plot Frequency Response
figure;
semilogx(results_hz, results_dB_SPL, '-o');
title('Frequency Response');
xlabel('Frequency (Hz)');
ylabel('Level (dB SPL)');
grid on;
