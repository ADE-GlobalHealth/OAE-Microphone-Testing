%% Noise Floor Analysis (Tests 2 & 4)
% Single silent capture - no tone present, so it's valid to read all 6
% DPOAE bins from the one FFT (unlike the tone-by-tone Frequency Response
% test, there's no risk of reading noise where a signal should be, since
% there IS no signal here by design - electronics-only floor and the
% self-noise floor are both "point this at a silent capture" tests).
%
% Reuse this for both Test 2 (electronics-only floor, mic disconnected)
% and Test 4 (silent self-noise floor, mic sealed in a quiet cavity) by
% pointing Signal at the relevant capture CSV and rerunning. When applied
% to a Test 4 capture, the dB SPL column below IS the EIN at that bin -
% same formula, just a different label depending on which capture you fed
% it.
Fs = 48000;   % Sampling frequency in Hz

% Load, Clip, and Window Signal
%file path is C:\Users\bspiller\Desktop\DaisyExamples\seed\Audio_Data_Collection
Signal = readmatrix("SPH0645_NoiseFloor.csv");
Clipped_Signal = Signal((end-32767:end),1);  % last 32768 samples
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

%% DPOAE Bin Levels
dpoae_bins_hz = [600, 1200, 2000, 3000, 4000, 6000];
bin_halfwidth = 2; % +-2 bins around each target frequency

% Sensitivity constant from Test 3 (pistonphone calibration) - counts per
% Pascal for this candidate at the locked gain setting. Set this before
% running; every dB SPL value below depends on it being accurate.
sensitivity_counts_per_Pa = 0.0031; %sph

fprintf('\n%-10s %-14s %-14s %-12s\n', 'Freq (Hz)', 'Level (counts)', 'Level (dBFS)', 'dB SPL');

for bin_hz = dpoae_bins_hz
    [~, bin_idx] = min(abs(f - bin_hz));
    band = max(1, bin_idx - bin_halfwidth) : min(length(P1), bin_idx + bin_halfwidth);
    bin_counts = sqrt(sum(power_spectrum(band))); % RMS amplitude in the band
    bin_dBFS = 20 * log10(bin_counts);

    pascals = bin_counts / sensitivity_counts_per_Pa;
    dB_SPL = 20 * log10(pascals / 20e-6);

    fprintf('%-10g %-14g %-14.2f %-12.2f\n', bin_hz, bin_counts, bin_dBFS, dB_SPL);
end
