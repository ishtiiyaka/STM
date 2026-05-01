%% ============================================================
%  Motor Fault Detection — Signal Analysis
%  Step 2: FFT, Envelope Spectrum, Sampling Rate Comparison,
%          Statistical Features, Fault Classification
%  UET Lahore, Special Topics in Mechatronics, Spring 2026
%% ============================================================
%
%  REQUIRES: signal_analysis.m  (this file)
%  Run AFTER collecting both healthy and faulty datasets with
%  data_acquisition.m
%
%  Files expected:
%    data_healthy_100Hz.mat,  data_healthy_500Hz.mat,  data_healthy_1000Hz.mat
%    data_faulty_100Hz.mat,   data_faulty_500Hz.mat,   data_faulty_1000Hz.mat
%% ============================================================

clear; clc; close all;

%% ─── LOAD DATA ───────────────────────────────────────────────
fprintf('=== Signal Analysis & Fault Detection ===\n\n');

rates     = [100, 500, 1000];
rate_tags = {'100Hz', '500Hz', '1000Hz'};
colors_h  = {'b', [0 0.6 1], [0 0.3 0.8]};   % healthy = blue shades
colors_f  = {'r', [1 0.4 0], [0.8 0 0]};      % faulty  = red shades

healthy = cell(1,3);
faulty  = cell(1,3);
have_faulty = false;

for i = 1:3
    fh = sprintf('data_healthy_%s.mat', rate_tags{i});
    ff = sprintf('data_faulty_%s.mat',  rate_tags{i});
    if exist(fh, 'file')
        tmp = load(fh); healthy{i} = tmp.data;
        fprintf('Loaded: %s  (%d samples, %.0f Hz actual)\n', ...
            fh, numel(healthy{i}.timestamp), healthy{i}.fs_actual);
    else
        fprintf('WARNING: %s not found — skipping.\n', fh);
    end
    if exist(ff, 'file')
        tmp = load(ff); faulty{i} = tmp.data;
        have_faulty = true;
        fprintf('Loaded: %s  (%d samples, %.0f Hz actual)\n', ...
            ff, numel(faulty{i}.timestamp), faulty{i}.fs_actual);
    end
end
fprintf('\n');

% Use 1000 Hz dataset for primary analysis
h = healthy{3};    % healthy @ 1000 Hz
Fs = h.fs_actual;
if isnan(Fs) || Fs < 50, Fs = 1000; end

%% ─────────────────────────────────────────────────────────────
%  SECTION 1: Sampling Rate Comparison
%% ─────────────────────────────────────────────────────────────
figure('Name','1 — Sampling Rate Comparison','Position',[50 50 1200 600]);
for i = 1:3
    if isempty(healthy{i}); continue; end
    d = healthy{i};
    t = (d.timestamp - d.timestamp(1)) / 1000;
    vib = sqrt(d.ax.^2 + d.ay.^2 + d.az.^2);
    show_n = min(numel(t), round(d.fs_actual * 0.5)); % show 0.5 s

    subplot(3,2, 2*i-1);
    plot(t(1:show_n), vib(1:show_n), 'Color', colors_h{i}, 'LineWidth', 0.8);
    xlabel('Time (s)'); ylabel('|acc| (g)');
    title(sprintf('Vibration — %d Hz (healthy)', rates(i)));
    grid on;

    subplot(3,2, 2*i);
    [f, mag] = fft_analysis(vib, d.fs_actual);
    plot(f, mag, 'Color', colors_h{i}, 'LineWidth', 0.8);
    xlabel('Frequency (Hz)'); ylabel('Amplitude (g)');
    title(sprintf('FFT — %d Hz', rates(i)));
    xlim([0 min(d.fs_actual/2, 500)]); grid on;
end
sgtitle('Effect of Sampling Rate on Vibration Signal and Spectrum');

%% ─────────────────────────────────────────────────────────────
%  SECTION 2: FFT Spectrum Analysis (Healthy vs Faulty)
%% ─────────────────────────────────────────────────────────────
figure('Name','2 — Spectrum Analysis','Position',[60 60 1200 700]);

t_h = (h.timestamp - h.timestamp(1)) / 1000;
vib_h = sqrt(h.ax.^2 + h.ay.^2 + h.az.^2);
[f_h, mag_vib_h] = fft_analysis(vib_h, Fs);
[~,  mag_c2_h]   = fft_analysis(h.curr2, Fs);

subplot(2,2,1);
plot(f_h, mag_vib_h, 'b', 'LineWidth', 1);
hold on;
[pks, locs] = findpeaks(mag_vib_h, f_h, 'MinPeakProminence', 0.003, 'NPeaks', 5);
plot(locs, pks, 'bv', 'MarkerFaceColor','b', 'MarkerSize', 7);
for k = 1:numel(pks)
    text(locs(k)+2, pks(k), sprintf('%.0fHz', locs(k)), 'FontSize', 8, 'Color','b');
end
xlabel('Frequency (Hz)'); ylabel('Amplitude (g)');
title('Vibration Spectrum — HEALTHY');
xlim([0 Fs/2]); grid on;

subplot(2,2,3);
plot(f_h, mag_c2_h, 'b', 'LineWidth', 1);
xlabel('Frequency (Hz)'); ylabel('Amplitude (A)');
title('Current Spectrum — Motor 2, HEALTHY');
xlim([0 min(Fs/2, 200)]); grid on;

if have_faulty && ~isempty(faulty{3})
    f_dat = faulty{3};
    Fs_f  = f_dat.fs_actual;
    if isnan(Fs_f) || Fs_f < 50; Fs_f = 1000; end

    vib_f = sqrt(f_dat.ax.^2 + f_dat.ay.^2 + f_dat.az.^2);
    [f_ff, mag_vib_f] = fft_analysis(vib_f, Fs_f);
    [~,   mag_c2_f]   = fft_analysis(f_dat.curr2, Fs_f);

    subplot(2,2,2);
    plot(f_ff, mag_vib_f, 'r', 'LineWidth', 1);
    hold on;
    [pks2, locs2] = findpeaks(mag_vib_f, f_ff, 'MinPeakProminence', 0.003, 'NPeaks', 5);
    plot(locs2, pks2, 'rv', 'MarkerFaceColor','r', 'MarkerSize', 7);
    for k = 1:numel(pks2)
        text(locs2(k)+2, pks2(k), sprintf('%.0fHz', locs2(k)), 'FontSize', 8, 'Color','r');
    end
    xlabel('Frequency (Hz)'); ylabel('Amplitude (g)');
    title('Vibration Spectrum — FAULTY');
    xlim([0 Fs_f/2]); grid on;

    subplot(2,2,4);
    plot(f_ff, mag_c2_f, 'r', 'LineWidth', 1);
    xlabel('Frequency (Hz)'); ylabel('Amplitude (A)');
    title('Current Spectrum — Motor 2, FAULTY');
    xlim([0 min(Fs_f/2, 200)]); grid on;
else
    subplot(2,2,2); text(0.3,0.5,'Faulty data not found','Units','normalized');
    subplot(2,2,4); text(0.3,0.5,'Faulty data not found','Units','normalized');
end
sgtitle('Spectrum Analysis — Healthy vs Faulty');

%% ─────────────────────────────────────────────────────────────
%  SECTION 3: Envelope Spectrum (Bearing Fault Detection)
%% ─────────────────────────────────────────────────────────────
figure('Name','3 — Envelope Spectrum','Position',[70 70 1200 600]);

datasets_env = {'HEALTHY', h};
if have_faulty && ~isempty(faulty{3})
    datasets_env{end+1} = 'FAULTY';
    datasets_env{end+1} = faulty{3};
end

n_env = numel(datasets_env)/2;
for k = 1:n_env
    lbl  = datasets_env{2*k-1};
    d    = datasets_env{2*k};
    Fs_k = d.fs_actual;
    if isnan(Fs_k) || Fs_k < 200
        fprintf('WARNING: %s data too low rate for envelope spectrum. Need >= 200 Hz.\n', lbl);
        continue
    end

    vib_k = sqrt(d.ax.^2 + d.ay.^2 + d.az.^2);
    vib_k = vib_k - mean(vib_k);

    % High-pass Butterworth 80 Hz to isolate bearing frequencies
    [b_hp, a_hp] = butter(4, 80 / (Fs_k/2), 'high');
    vib_hp = filtfilt(b_hp, a_hp, vib_k);

    % Envelope via Hilbert transform
    env_sig = abs(hilbert(vib_hp));
    env_sig = env_sig - mean(env_sig);

    [f_env, mag_env] = fft_analysis(env_sig, Fs_k);

    clr = 'b';
    if strcmp(lbl,'FAULTY'), clr = 'r'; end

    subplot(n_env, 2, 2*k-1);
    t_k = (1:numel(vib_hp)) / Fs_k;
    show_n = min(numel(t_k), round(Fs_k * 0.3));
    plot(t_k(1:show_n), vib_hp(1:show_n), clr, 'LineWidth', 0.7);
    hold on;
    plot(t_k(1:show_n), env_sig(1:show_n), 'k', 'LineWidth', 1.5);
    legend('HP filtered vib.', 'Envelope', 'Location', 'best');
    xlabel('Time (s)'); ylabel('Amplitude (g)');
    title(sprintf('HP Vibration + Envelope — %s', lbl));
    grid on;

    subplot(n_env, 2, 2*k);
    plot(f_env, mag_env, clr, 'LineWidth', 1);
    hold on;
    [pke, lce] = findpeaks(mag_env, f_env, 'MinPeakProminence', 5e-4, 'NPeaks', 5);
    plot(lce, pke, [clr 'v'], 'MarkerFaceColor', clr, 'MarkerSize', 7);
    for ki = 1:numel(pke)
        text(lce(ki)+1, pke(ki)*1.08, sprintf('%.1fHz', lce(ki)), 'FontSize', 8, 'Color', clr);
    end
    xlabel('Frequency (Hz)'); ylabel('Envelope amplitude');
    title(sprintf('Envelope Spectrum — %s', lbl));
    xlim([0 min(Fs_k/4, 250)]); grid on;
end
sgtitle('Envelope Spectrum Analysis (Bearing Fault Indicator)');

%% ─────────────────────────────────────────────────────────────
%  SECTION 4: Statistical Feature Comparison Table
%% ─────────────────────────────────────────────────────────────
fprintf('=== Statistical Feature Comparison ===\n\n');
fprintf('%-22s  %12s  %12s\n', 'Feature', 'HEALTHY', 'FAULTY');
fprintf('%s\n', repmat('-', 1, 50));

function_names = {'Vib RMS (g)', 'Vib Peak (g)', 'Crest Factor', ...
                  'Kurtosis (vib)', 'Current M2 mean (A)', 'Current M2 RMS (A)', ...
                  'Current M2 std (A)', 'Temp mean (°C)', 'Temp max (°C)'};

feats_h = compute_features(h);
if have_faulty && ~isempty(faulty{3})
    feats_f = compute_features(faulty{3});
else
    feats_f = nan(size(feats_h));
end

for fi = 1:numel(function_names)
    fprintf('%-22s  %12.4f  %12.4f\n', function_names{fi}, feats_h(fi), feats_f(fi));
end

%% ─────────────────────────────────────────────────────────────
%  SECTION 5: Feature Bar Chart (Visual Comparison)
%% ─────────────────────────────────────────────────────────────
if have_faulty && ~isempty(faulty{3})
    figure('Name','5 — Feature Comparison','Position',[80 80 1000 500]);
    n_feats_show = 6;  % first 6 features for bar chart
    X = categorical(function_names(1:n_feats_show));
    X = reordercats(X, function_names(1:n_feats_show));
    bar_data = [feats_h(1:n_feats_show); feats_f(1:n_feats_show)]';
    h_bar = bar(X, bar_data);
    h_bar(1).FaceColor = [0.2 0.4 0.8];
    h_bar(2).FaceColor = [0.9 0.2 0.2];
    legend('Healthy', 'Faulty', 'Location', 'northwest');
    ylabel('Feature value');
    title('Statistical Feature Comparison — Healthy vs Faulty');
    grid on; xtickangle(20);
end

fprintf('\nSignal analysis complete.\n');

%% ─────────────────────────────────────────────────────────────
%  LOCAL HELPER FUNCTIONS
%% ─────────────────────────────────────────────────────────────

function [f, mag] = fft_analysis(signal, fs)
% Compute single-sided amplitude spectrum with Hann window
    N      = numel(signal);
    sig_dc = signal(:)' - mean(signal);
    win    = hann(N)';
    Y      = fft(sig_dc .* win, N);
    mag    = (2/sum(win)) * abs(Y(1:floor(N/2)+1));
    f      = (0:floor(N/2)) * (fs / N);
end

function feats = compute_features(d)
% Extract statistical features from a data struct
    vib  = sqrt(d.ax.^2 + d.ay.^2 + d.az.^2);
    vib  = vib - mean(vib);
    vib_rms   = rms(vib);
    vib_peak  = max(abs(vib));
    crest_f   = vib_peak / (vib_rms + eps);
    kurt_v    = kurtosis(vib);
    c2_mean   = mean(d.curr2);
    c2_rms    = rms(d.curr2);
    c2_std    = std(d.curr2);
    temp_mean = mean(d.temperature);
    temp_max  = max(d.temperature);

    feats = [vib_rms, vib_peak, crest_f, kurt_v, ...
             c2_mean, c2_rms, c2_std, temp_mean, temp_max];
end
