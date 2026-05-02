%% ============================================================
%  Motor Fault Detection — Data Acquisition
%  Step 1: Logs sensor data from Arduino at 3 sampling rates
%  UET Lahore, Special Topics in Mechatronics, Spring 2026
%% ============================================================
%
%  HOW TO USE:
%    1. Upload motor_fault.ino to Arduino
%    2. Change COM_PORT below to your Arduino's COM port
%       (Check: Arduino IDE → Tools → Port)
%    3. Run this script
%    4. When prompted, rotate the potentiometer to each position
%    5. Three .mat files are saved for signal_analysis.m
%
%  REQUIRED: MATLAB R2019b+ (uses serialport object)
%% ============================================================

clear; clc; close all;

%% ─── USER SETTINGS ──────────────────────────────────────────
COM_PORT      = 'COM3';       % <-- Change to your port (e.g. COM5, /dev/ttyUSB0)
BAUD_RATE     = 115200;
LOG_DURATION  = 30;           % seconds per sampling rate
CONDITION     = 'healthy';    % 'healthy' or 'faulty' — used in filename

% Output filenames (3 rates × 1 condition = 3 files)
save_names = { ...
    sprintf('data_%s_100Hz.mat',  CONDITION), ...
    sprintf('data_%s_500Hz.mat',  CONDITION), ...
    sprintf('data_%s_1000Hz.mat', CONDITION)  ...
};
target_rates = [100, 500, 1000];
pot_positions = {'LOW (0–33%)', 'MID (34–66%)', 'HIGH (67–100%)'};

%% ─── CONNECT ────────────────────────────────────────────────
fprintf('=== Motor Fault Detection — Data Acquisition ===\n\n');
fprintf('Connecting to Arduino on %s at %d baud...\n', COM_PORT, BAUD_RATE);

try
    s = serialport(COM_PORT, BAUD_RATE);
catch e
    error('Cannot open %s. Check port and that Arduino IDE serial monitor is closed.\n%s', ...
          COM_PORT, e.message);
end
configureTerminator(s, "LF");
flush(s);
pause(2.5);  % wait for Arduino reset + calibration message
fprintf('Connected!\n\n');

% Drain calibration messages
while s.NumBytesAvailable > 0
    line = readline(s);
    fprintf('Arduino: %s\n', strtrim(line));
    pause(0.05);
end

%% ─── ACQUIRE DATA AT EACH SAMPLE RATE ───────────────────────
for sr_idx = 1:3
    target_fs = target_rates(sr_idx);
    fprintf('\n────────────────────────────────────────\n');
    fprintf('  Run %d/3  |  Sample Rate: %d Hz\n', sr_idx, target_fs);
    fprintf('  Pot position: %s\n', pot_positions{sr_idx});
    fprintf('  Data file: %s\n', save_names{sr_idx});
    fprintf('────────────────────────────────────────\n');
    fprintf('  ▶  Rotate potentiometer to position %d/3, then press Enter...\n', sr_idx);
    pause;

    % Initialize data struct
    D.condition   = CONDITION;
    D.target_fs   = target_fs;
    D.timestamp   = [];
    D.curr1       = [];   % Motor 1 current [A]
    D.curr2       = [];   % Motor 2 current [A]
    D.temperature = [];   % [°C]
    D.ax          = [];   % Acceleration X [g]
    D.ay          = [];
    D.az          = [];
    D.gx          = [];   % Angular rate X [°/s]
    D.gy          = [];
    D.gz          = [];
    D.fs_reported = [];   % Sample rate reported by Arduino

    flush(s);
    t_start  = tic;
    n_ok     = 0;
    n_bad    = 0;
    prev_pct = -1;

    fprintf('  Logging for %d seconds', LOG_DURATION);

    while toc(t_start) < LOG_DURATION
        if s.NumBytesAvailable > 0
            try
                line = strtrim(readline(s));
                % Skip blanks, comments, header
                if isempty(line) || line(1)=='#' || startsWith(line,'timestamp')
                    continue
                end
                vals = str2double(strsplit(line, ','));
                if numel(vals) ~= 11 || any(isnan(vals))
                    n_bad = n_bad + 1;
                    continue
                end
                D.timestamp(end+1)   = vals(1);
                D.curr1(end+1)       = vals(2);
                D.curr2(end+1)       = vals(3);
                D.temperature(end+1) = vals(4);
                D.ax(end+1)          = vals(5);
                D.ay(end+1)          = vals(6);
                D.az(end+1)          = vals(7);
                D.gx(end+1)          = vals(8);
                D.gy(end+1)          = vals(9);
                D.gz(end+1)          = vals(10);
                D.fs_reported(end+1) = vals(11);
                n_ok = n_ok + 1;
            catch
                n_bad = n_bad + 1;
            end
        end

        % Progress dots
        pct = floor(toc(t_start) / LOG_DURATION * 20);
        if pct > prev_pct
            fprintf('.');
            prev_pct = pct;
        end
    end

    fprintf('\n');

    % Compute actual achieved sample rate
    if numel(D.timestamp) > 1
        elapsed_s = (D.timestamp(end) - D.timestamp(1)) / 1000;
        D.fs_actual = (numel(D.timestamp) - 1) / max(elapsed_s, 0.001);
    else
        D.fs_actual = NaN;
    end

    fprintf('  Collected %d samples (%.0f bad lines dropped)\n', n_ok, n_bad);
    fprintf('  Actual sample rate: %.1f Hz\n', D.fs_actual);
    fprintf('  Temperature range: %.1f – %.1f °C\n', min(D.temperature), max(D.temperature));
    fprintf('  Current M1: %.3f A mean  |  M2: %.3f A mean\n', ...
        mean(D.curr1), mean(D.curr2));

    % Save
    data = D; %#ok<NASGU>
    save(save_names{sr_idx}, 'data');
    fprintf('  Saved → %s\n', save_names{sr_idx});
end

%% ─── CLEANUP ────────────────────────────────────────────────
clear s;
fprintf('\n✓ Data acquisition complete!\n');
fprintf('Now run signal_analysis.m or fault_detection_realtime.m\n\n');

%% ─── QUICK PREVIEW PLOT ──────────────────────────────────────
D_preview = load(save_names{2});  % load 500 Hz file for preview
dp = D_preview.data;
t_s = (dp.timestamp - dp.timestamp(1)) / 1000;
vib = sqrt(dp.ax.^2 + dp.ay.^2 + dp.az.^2);

figure('Name', 'Quick Preview — 500 Hz Data', 'Position', [100 100 1100 600]);
subplot(3,1,1);
plot(t_s, dp.curr1, 'b', t_s, dp.curr2, 'r', 'LineWidth', 0.8);
legend('Motor 1 (ref)', 'Motor 2 (test)', 'Location', 'best');
ylabel('Current (A)'); title('Motor Currents'); grid on;

subplot(3,1,2);
plot(t_s, vib, 'm', 'LineWidth', 0.8);
ylabel('Vibration (g)'); title('Vibration Magnitude |acc|'); grid on;

subplot(3,1,3);
plot(t_s, dp.temperature, 'r', 'LineWidth', 1.2);
ylabel('Temp (°C)'); xlabel('Time (s)'); title('Motor Temperature'); grid on;

sgtitle(sprintf('Quick Preview — Condition: %s — 500 Hz', CONDITION));
