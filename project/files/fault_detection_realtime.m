%% ============================================================
%  Motor Fault Detection — Real-Time Monitoring & Classification
%  Step 3: Live dashboard, FFT, envelope spectrum, fault alerts
%  UET Lahore, Special Topics in Mechatronics, Spring 2026
%% ============================================================
%
%  HOW TO USE:
%    1. Run data_acquisition.m in HEALTHY condition first to
%       establish baselines (the script auto-computes them).
%    2. Close that script, then run THIS script.
%    3. The dashboard shows live data + fault status.
%    4. To simulate faults:
%         Overload   → add load to Motor 2 shaft
%         Overheating→ will rise naturally; or block airflow
%         Bearing    → attach a small unbalanced weight to shaft
%         Imbalance  → same as above (heavier weight)
%    5. Close the figure to stop.
%
%  REQUIRES: MATLAB R2019b+, Signal Processing Toolbox
%% ============================================================

clear; clc; close all;

%% ─── USER SETTINGS ──────────────────────────────────────────
COM_PORT     = 'COM3';     % <-- Change to your port
BAUD_RATE    = 115200;
BUFFER_SIZE  = 1024;       % FFT window size (must be power of 2)
UPDATE_EVERY = 30;         % Redraw plot every N new samples

%% ─── FAULT THRESHOLDS ────────────────────────────────────────
%  Adjust after observing healthy baseline values
TH.temp_warn      = 45.0;  % °C  — warning
TH.temp_fault     = 60.0;  % °C  — overheating fault
TH.curr_warn      = 1.2;   % A   — high current warning
TH.curr_overload  = 2.0;   % A   — overload fault
TH.vib_rms_warn   = 0.30;  % g   — vibration warning
TH.vib_rms_fault  = 0.80;  % g   — bearing/imbalance fault
TH.crest_warn     = 3.5;   —      % bearing impact indicator
TH.crest_fault    = 5.0;
TH.kurtosis_fault = 5.0;   % kurtosis > 5 → impulsive → bearing

%% ─── LOAD BASELINE FROM HEALTHY FILE (if available) ──────────
baseline_file = 'data_healthy_1000Hz.mat';
if exist(baseline_file, 'file')
    tmp = load(baseline_file);
    bsl = tmp.data;
    vib_bsl = sqrt(bsl.ax.^2 + bsl.ay.^2 + bsl.az.^2);
    TH.vib_rms_warn   = 2.0 * rms(vib_bsl - mean(vib_bsl));
    TH.vib_rms_fault  = 4.0 * rms(vib_bsl - mean(vib_bsl));
    TH.curr_warn      = 1.3 * mean(bsl.curr2);
    TH.curr_overload  = 2.0 * mean(bsl.curr2);
    TH.temp_warn      = max(bsl.temperature) + 10;
    TH.temp_fault     = max(bsl.temperature) + 25;
    fprintf('Thresholds loaded from baseline: %s\n\n', baseline_file);
else
    fprintf('No baseline file found — using default thresholds.\n\n');
end

%% ─── CONNECT TO ARDUINO ─────────────────────────────────────
fprintf('Connecting to %s...\n', COM_PORT);
try
    s = serialport(COM_PORT, BAUD_RATE);
catch e
    error('Cannot open %s. Ensure Arduino IDE serial monitor is closed.\n%s', ...
          COM_PORT, e.message);
end
configureTerminator(s, "LF");
flush(s); pause(2.5);
fprintf('Connected! Starting real-time monitoring...\n');
fprintf('Close the figure window to stop.\n\n');

% Drain startup messages
while s.NumBytesAvailable > 0
    readline(s);
    pause(0.02);
end

%% ─── CIRCULAR BUFFERS ────────────────────────────────────────
buf_t    = zeros(1, BUFFER_SIZE);
buf_c1   = zeros(1, BUFFER_SIZE);
buf_c2   = zeros(1, BUFFER_SIZE);
buf_temp = zeros(1, BUFFER_SIZE);
buf_ax   = zeros(1, BUFFER_SIZE);
buf_ay   = zeros(1, BUFFER_SIZE);
buf_az   = zeros(1, BUFFER_SIZE);
ptr      = 1;   % write pointer
n_total  = 0;
Fs_live  = 500; % updated from incoming data

%% ─── FIGURE LAYOUT ──────────────────────────────────────────
fig = figure('Name', 'Motor Fault Detection — Live Dashboard', ...
    'Position', [30 30 1400 860], 'Color', [0.08 0.08 0.10]);
set(fig, 'CloseRequestFcn', @(~,~) set(fig, 'UserData', 'stop'));

ax_vib   = subplot(3,3,1);
ax_curr  = subplot(3,3,2);
ax_temp  = subplot(3,3,3);
ax_fft   = subplot(3,3,4);
ax_env   = subplot(3,3,5);
ax_c_fft = subplot(3,3,6);
ax_stat  = subplot(3,3,[7 8 9]);

all_ax = [ax_vib, ax_curr, ax_temp, ax_fft, ax_env, ax_c_fft, ax_stat];
for ax = all_ax
    set(ax, 'Color', [0.10 0.10 0.13], ...
            'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
            'GridColor', [0.25 0.25 0.28], 'GridAlpha', 0.4);
    grid(ax, 'on');
end
set(ax_stat, 'Visible', 'off');

%% ─── FAULT LOG (in-memory) ───────────────────────────────────
fault_log = {};

%% ─── MAIN LOOP ───────────────────────────────────────────────
t_start  = tic;

while ishandle(fig) && ~strcmp(get(fig, 'UserData'), 'stop')

    % ── Read all available serial lines ──
    lines_read = 0;
    while s.NumBytesAvailable > 0 && lines_read < 50
        try
            line = strtrim(readline(s));
            lines_read = lines_read + 1;
            if isempty(line) || line(1)=='#' || startsWith(line,'timestamp')
                continue
            end
            vals = str2double(strsplit(line, ','));
            if numel(vals) ~= 11 || any(isnan(vals)); continue; end

            buf_t(ptr)    = vals(1);
            buf_c1(ptr)   = vals(2);
            buf_c2(ptr)   = vals(3);
            buf_temp(ptr) = vals(4);
            buf_ax(ptr)   = vals(5);
            buf_ay(ptr)   = vals(6);
            buf_az(ptr)   = vals(7);
            Fs_live       = vals(11);
            ptr = mod(ptr, BUFFER_SIZE) + 1;
            n_total = n_total + 1;
        catch
            % ignore parse errors
        end
    end

    % ── Update dashboard every UPDATE_EVERY samples ──
    if mod(n_total, UPDATE_EVERY) == 0 && n_total >= BUFFER_SIZE

        % Unwrap circular buffer chronologically
        idx   = [ptr:BUFFER_SIZE, 1:ptr-1];
        t_buf = buf_t(idx);
        c1    = buf_c1(idx);
        c2    = buf_c2(idx);
        temp  = buf_temp(idx);
        ax_d  = buf_ax(idx);
        ay_d  = buf_ay(idx);
        az_d  = buf_az(idx);
        t_rel = (t_buf - t_buf(1)) / 1000;   % ms → s
        vib   = sqrt(ax_d.^2 + ay_d.^2 + az_d.^2);

        % ── Compute features ──
        vib_dc   = vib - mean(vib);
        vib_rms  = rms(vib_dc);
        vib_peak = max(abs(vib_dc));
        cf       = vib_peak / (vib_rms + eps);
        kurt_v   = kurtosis(vib_dc);
        curr_now = c2(end);
        temp_now = temp(end);

        % ── Vibration plot ──
        plot(ax_vib, t_rel, vib, 'Color', [0.35 0.75 1], 'LineWidth', 0.7);
        yline(ax_vib, TH.vib_rms_warn,  '--', 'Color', [1 0.8 0], 'LineWidth', 1.2);
        yline(ax_vib, TH.vib_rms_fault, '--', 'Color', [1 0.3 0.3], 'LineWidth', 1.5);
        title(ax_vib, sprintf('Vibration  RMS=%.3fg  CF=%.1f  Kurt=%.1f', ...
              vib_rms, cf, kurt_v), 'Color', 'w', 'FontSize', 10);
        xlabel(ax_vib, 'Time (s)', 'Color', [0.7 0.7 0.7]);
        ylabel(ax_vib, 'g', 'Color', [0.7 0.7 0.7]);

        % ── Current plot ──
        plot(ax_curr, t_rel, c1, 'Color', [0.4 0.9 0.5], 'LineWidth', 0.7);
        hold(ax_curr, 'on');
        plot(ax_curr, t_rel, c2, 'Color', [1 0.45 0.45], 'LineWidth', 0.7);
        yline(ax_curr, TH.curr_overload, '--', 'Color', [1 0.3 0.3], 'LineWidth', 1.5);
        hold(ax_curr, 'off');
        legend(ax_curr, 'M1 healthy', 'M2 test', ...
               'Color', [0.12 0.12 0.15], 'TextColor', 'w', 'FontSize', 8);
        title(ax_curr, sprintf('Current  M1=%.2fA  M2=%.2fA', mean(c1), mean(c2)), ...
              'Color', 'w', 'FontSize', 10);
        ylabel(ax_curr, 'A', 'Color', [0.7 0.7 0.7]);

        % ── Temperature plot ──
        plot(ax_temp, t_rel, temp, 'Color', [1 0.55 0.3], 'LineWidth', 1.2);
        hold(ax_temp, 'on');
        yline(ax_temp, TH.temp_warn,  '--', 'Color', [1 0.8 0], 'LineWidth', 1.2);
        yline(ax_temp, TH.temp_fault, '--', 'Color', [1 0.3 0.3], 'LineWidth', 1.5);
        hold(ax_temp, 'off');
        title(ax_temp, sprintf('Temperature: %.1f°C', temp_now), ...
              'Color', 'w', 'FontSize', 10);
        ylabel(ax_temp, '°C', 'Color', [0.7 0.7 0.7]);

        % ── Vibration FFT ──
        [f_ax, mag_vib_fft] = local_fft(vib_dc, Fs_live);
        plot(ax_fft, f_ax, mag_vib_fft, 'Color', [0.5 0.7 1], 'LineWidth', 0.8);
        hold(ax_fft, 'on');
        [pks, lcs] = findpeaks(mag_vib_fft, f_ax, 'MinPeakProminence', 0.005, 'NPeaks', 4);
        plot(ax_fft, lcs, pks, 'rv', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
        for k = 1:numel(pks)
            text(ax_fft, lcs(k)+2, pks(k), sprintf('%.0f', lcs(k)), ...
                 'Color', 'r', 'FontSize', 8);
        end
        hold(ax_fft, 'off');
        xlim(ax_fft, [0 min(Fs_live/2, 400)]);
        title(ax_fft, 'Vibration Spectrum', 'Color', 'w', 'FontSize', 10);
        xlabel(ax_fft, 'Hz', 'Color', [0.7 0.7 0.7]);
        ylabel(ax_fft, 'g', 'Color', [0.7 0.7 0.7]);

        % ── Envelope Spectrum ──
        if Fs_live >= 200
            [b_hp, a_hp] = butter(4, 80 / (Fs_live/2), 'high');
            vib_hp  = filtfilt(b_hp, a_hp, vib_dc);
            env_sig = abs(hilbert(vib_hp));
            env_sig = env_sig - mean(env_sig);
            [f_env, mag_env] = local_fft(env_sig, Fs_live);
            plot(ax_env, f_env, mag_env, 'Color', [0.8 0.5 1], 'LineWidth', 0.8);
            hold(ax_env, 'on');
            [epks, elcs] = findpeaks(mag_env, f_env, 'MinPeakProminence', 2e-4, 'NPeaks', 4);
            plot(ax_env, elcs, epks, 'yv', 'MarkerFaceColor', 'y', 'MarkerSize', 6);
            hold(ax_env, 'off');
            xlim(ax_env, [0 min(Fs_live/4, 200)]);
        end
        title(ax_env, 'Envelope Spectrum (bearing)', 'Color', 'w', 'FontSize', 10);
        xlabel(ax_env, 'Hz', 'Color', [0.7 0.7 0.7]);

        % ── Current FFT ──
        [f_cx, mag_c2f] = local_fft(c2 - mean(c2), Fs_live);
        plot(ax_c_fft, f_cx, mag_c2f, 'Color', [1 0.55 0.55], 'LineWidth', 0.8);
        xlim(ax_c_fft, [0 min(Fs_live/2, 200)]);
        title(ax_c_fft, 'Current Spectrum M2', 'Color', 'w', 'FontSize', 10);
        xlabel(ax_c_fft, 'Hz', 'Color', [0.7 0.7 0.7]);
        ylabel(ax_c_fft, 'A', 'Color', [0.7 0.7 0.7]);

        % ── Fault Classification ──
        [status_lines, any_fault] = classify_faults( ...
            vib_rms, cf, kurt_v, curr_now, temp_now, TH);

        % Log new faults
        for k = 1:numel(status_lines)
            if contains(status_lines{k}, 'FAULT')
                fault_log{end+1} = sprintf('[t=%.1fs] %s', toc(t_start), status_lines{k});
                fprintf('%s\n', fault_log{end});
            end
        end

        % ── Status Panel ──
        cla(ax_stat);
        set(ax_stat, 'Visible', 'off');

        % Panel background colour
        if any_fault
            bg_col = [0.20 0.06 0.06];
        else
            bg_col = [0.06 0.15 0.08];
        end
        annotation_text_box(ax_stat, bg_col);

        elapsed = toc(t_start);
        title(ax_stat, sprintf('FAULT STATUS   ▐ t = %.0fs  |  Fs = %.0f Hz  |  n = %d', ...
              elapsed, Fs_live, n_total), 'Color', 'w', 'FontSize', 11, 'FontWeight', 'normal');

        y_pos = 0.78;
        for k = 1:numel(status_lines)
            sl = status_lines{k};
            if contains(sl, 'FAULT')
                clr = [1.0 0.28 0.28];
            elseif contains(sl, 'WARNING')
                clr = [1.0 0.82 0.15];
            else
                clr = [0.35 0.95 0.45];
            end
            text(ax_stat, 0.03, y_pos, sl, 'Color', clr, ...
                 'FontSize', 11.5, 'FontWeight', 'bold', 'Units', 'normalized');
            y_pos = y_pos - 0.20;
        end

        drawnow limitrate;
    end

    pause(0.001);
end

%% ─── SAVE SESSION LOG ────────────────────────────────────────
if ~isempty(fault_log)
    log_file = sprintf('fault_log_%s.txt', datestr(now,'yyyymmdd_HHMMSS'));
    fid = fopen(log_file, 'w');
    for k = 1:numel(fault_log)
        fprintf(fid, '%s\n', fault_log{k});
    end
    fclose(fid);
    fprintf('\nFault log saved → %s\n', log_file);
end

clear s;
fprintf('Monitoring stopped. Total samples: %d\n', n_total);

%% ─────────────────────────────────────────────────────────────
%  LOCAL HELPER FUNCTIONS
%% ─────────────────────────────────────────────────────────────

function [f, mag] = local_fft(signal, fs)
    N   = numel(signal);
    win = hann(N)';
    Y   = fft((signal(:)' - mean(signal)) .* win, N);
    mag = (2/sum(win)) * abs(Y(1:floor(N/2)+1));
    f   = (0:floor(N/2)) * (fs / N);
end

function [lines, any_fault] = classify_faults(vib_rms, cf, kurt, curr, temp, TH)
    lines     = {};
    any_fault = false;

    % ── Temperature ──
    if temp > TH.temp_fault
        lines{end+1} = sprintf('[FAULT] OVERHEATING  Temp = %.1f°C  (limit %.0f°C)', ...
                               temp, TH.temp_fault);
        any_fault = true;
    elseif temp > TH.temp_warn
        lines{end+1} = sprintf('[WARNING] High temp  %.1f°C', temp);
    else
        lines{end+1} = sprintf('[OK]  Temperature  %.1f°C', temp);
    end

    % ── Overload ──
    if curr > TH.curr_overload
        lines{end+1} = sprintf('[FAULT] OVERLOAD  Current = %.3f A  (limit %.1f A)', ...
                               curr, TH.curr_overload);
        any_fault = true;
    elseif curr > TH.curr_warn
        lines{end+1} = sprintf('[WARNING] High current  %.3f A', curr);
    else
        lines{end+1} = sprintf('[OK]  Current  %.3f A', curr);
    end

    % ── Bearing fault (impulsive, high kurtosis + high CF) ──
    if kurt > TH.kurtosis_fault && cf > TH.crest_fault
        lines{end+1} = sprintf('[FAULT] BEARING  Kurtosis = %.1f  CF = %.1f', kurt, cf);
        any_fault = true;
    elseif cf > TH.crest_warn
        lines{end+1} = sprintf('[WARNING] Impulsive vib  CF = %.1f', cf);
    else
        % ── Imbalance (high RMS but low kurtosis) ──
        if vib_rms > TH.vib_rms_fault
            lines{end+1} = sprintf('[FAULT] IMBALANCE/MISALIGN  Vib RMS = %.4f g', vib_rms);
            any_fault = true;
        elseif vib_rms > TH.vib_rms_warn
            lines{end+1} = sprintf('[WARNING] Elevated vibration  RMS = %.4f g', vib_rms);
        else
            lines{end+1} = sprintf('[OK]  Vibration  RMS = %.4f g', vib_rms);
        end
    end
end

function annotation_text_box(ax, bg_col)
% Draw a background rectangle behind the status panel
    pos = get(ax, 'Position');
    annotation('rectangle', pos, 'FaceColor', bg_col, ...
        'FaceAlpha', 0.9, 'EdgeColor', 'none');
end
