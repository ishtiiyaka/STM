import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
from collections import deque
import numpy as np

# ─── CONFIGURATION ────────────────────────────────────────────────────────────
COM_PORT     = 'COM6'       # Update if your port changes
BAUD_RATE    = 921600       # Must match sensor_mpu6050.ino Serial.begin(921600)
MAX_POINTS   = 1000         # Time-domain window: ~1.5 s at ~670 Hz effective ODR
FFT_SIZE     = 1024         # FFT window: 1024 samples
SAMPLE_RATE  = 1000         # Hz — matches SMPLRT_DIV=0 / DLPF_CFG=1 in .ino
ACCEL_LIMITS = (-2, 2)      # Expected g forces for time-domain display
GYRO_LIMITS  = (-250, 250)  # Expected °/s for time-domain display
FPS_INTERVAL = 30           # Milliseconds between plot redraws (~33 fps)

# ─── SERIAL CONNECTION ────────────────────────────────────────────────────────
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"🛰  Telemetry Linked: {COM_PORT} at {BAUD_RATE} baud")
except Exception as e:
    print(f"❌ Serial error: {e}")
    exit()

# ─── DATA CONTAINERS ──────────────────────────────────────────────────────────
# Time-domain ring buffers (oldest data auto-deleted)
data_ax = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
data_ay = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
data_az = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
data_gx = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
data_gy = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
data_gz = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)

# FFT accumulation buffer — fills to FFT_SIZE then is transformed
fft_buf_az = deque(maxlen=FFT_SIZE)

# Latest HUD values
latest_ax = latest_ay = latest_az = 0.0
latest_gx = latest_gy = latest_gz = 0.0

# FFT display arrays (pre-allocated, updated when buffer is full)
fft_freqs = np.fft.rfftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)   # 0 … 500 Hz
fft_mag   = np.zeros(len(fft_freqs))

# Shaft fundamental frequency detected from FFT (Hz); 0 = not yet detected
shaft_f1 = 0.0
# Rectangular window (= no windowing) — raw FFT, no spectral shaping
rect_window = np.ones(FFT_SIZE, dtype=np.float32)


# ─── UI LAYOUT & STYLING (OBSIDIAN PULSE AESTHETIC) ───────────────────────────
plt.style.use('dark_background')
fig = plt.figure(figsize=(14, 10), facecolor='#0A0A0A')
fig.canvas.manager.set_window_title("MPU6050 — Predictive Maintenance Monitor")

# 3-row layout:  row 0 = Accel time,  row 1 = Gyro time,  row 2 = FFT spectrum
gs = fig.add_gridspec(3, 1, hspace=0.42, top=0.95, bottom=0.06,
                      left=0.07, right=0.97)
ax_accel = fig.add_subplot(gs[0])
ax_gyro  = fig.add_subplot(gs[1])
ax_fft   = fig.add_subplot(gs[2])


def style_panel(ax, title, y_limits, ylabel=''):
    ax.set_facecolor('#050508')
    ax.set_title(title, color='#FFFFFF', fontweight='900', fontsize=12, pad=5)
    ax.set_ylim(*y_limits)
    ax.set_ylabel(ylabel, color='#666666', fontsize=9)
    ax.grid(True, color='#2A2A2A', linewidth=0.8, alpha=0.7)
    for spine in ax.spines.values():
        spine.set_color('#2A2A2A')
    ax.tick_params(colors='#555555', labelsize=8)


style_panel(ax_accel, "LINEAR ACCELERATION  (g)", ACCEL_LIMITS, 'g')
style_panel(ax_gyro,  "ROTATIONAL VELOCITY  (°/s)", GYRO_LIMITS, '°/s')
style_panel(ax_fft,   "VIBRATION SPECTRUM — AZ axis  [Hann FFT, 1024-pt]",
            (0, 0.05), 'Amplitude (g)')

ax_fft.set_xlabel('Frequency (Hz)', color='#666666', fontsize=9)
ax_fft.set_xlim(0, SAMPLE_RATE / 2)  # 0–500 Hz
ax_fft.xaxis.set_major_locator(ticker.MultipleLocator(50))
ax_fft.xaxis.set_minor_locator(ticker.MultipleLocator(10))

# ─── LINE MAKERS ──────────────────────────────────────────────────────────────
x_time = np.arange(MAX_POINTS)
x_fft  = fft_freqs


def make_raw_line(target_ax, x_data, y_data, color, label):
    """Plain line — no glow, no smoothing, no path effects."""
    l, = target_ax.plot(x_data, y_data, color=color,
                        linewidth=0.8, label=label,
                        solid_capstyle='butt', animated=True)
    return l


# Time-domain lines — raw, thin, no effects
line_ax_t = make_raw_line(ax_accel, x_time, [0] * MAX_POINTS, '#FF4466', 'AX')
line_ay_t = make_raw_line(ax_accel, x_time, [0] * MAX_POINTS, '#44FF88', 'AY')
line_az_t = make_raw_line(ax_accel, x_time, [0] * MAX_POINTS, '#44DDFF', 'AZ')
line_gx_t = make_raw_line(ax_gyro,  x_time, [0] * MAX_POINTS, '#FF4466', 'GX')
line_gy_t = make_raw_line(ax_gyro,  x_time, [0] * MAX_POINTS, '#44FF88', 'GY')
line_gz_t = make_raw_line(ax_gyro,  x_time, [0] * MAX_POINTS, '#44DDFF', 'GZ')

# FFT spectrum line — raw rectangular-window spectrum
line_fft, = ax_fft.plot(x_fft, fft_mag, color='#FFD700',
                         linewidth=0.8, animated=True,
                         label='AZ spectrum (rect window)')

# Harmonic marker lines (1×, 2×, 3× shaft fundamental) — invisible until f1 detected
harm_colors = ['#FF4500', '#FF8C00', '#FFD700']
harm_lines  = [ax_fft.axvline(x=0, color=c, linewidth=1.0, linestyle='--',
                               alpha=0.0, animated=True)
               for c in harm_colors]
harm_labels = [ax_fft.text(0, 0, '', color=c, fontsize=8,
                            fontweight='bold', fontfamily='monospace',
                            animated=True)
               for c in harm_colors]

# Peak label on FFT
peak_marker, = ax_fft.plot([], [], 'v', color='#FF4500',
                            markersize=7, animated=True)
peak_text = ax_fft.text(0, 0, '', color='#FF4500', fontsize=9,
                         fontweight='bold', fontfamily='monospace',
                         animated=True)

# ─── HUD ELEMENTS ─────────────────────────────────────────────────────────────
_hf = {'family': 'monospace', 'fontweight': 'bold', 'fontsize': 10}
txt_ax_v = ax_accel.text(0.012, 0.88, 'AX: +0.000 g', transform=ax_accel.transAxes, color='#FF0055', **_hf)
txt_ay_v = ax_accel.text(0.012, 0.76, 'AY: +0.000 g', transform=ax_accel.transAxes, color='#00FF66', **_hf)
txt_az_v = ax_accel.text(0.012, 0.64, 'AZ: +0.000 g', transform=ax_accel.transAxes, color='#00E5FF', **_hf)
txt_gx_v = ax_gyro.text(0.012, 0.88,  'GX: +000.0 °/s', transform=ax_gyro.transAxes,  color='#FF0055', **_hf)
txt_gy_v = ax_gyro.text(0.012, 0.76,  'GY: +000.0 °/s', transform=ax_gyro.transAxes,  color='#00FF66', **_hf)
txt_gz_v = ax_gyro.text(0.012, 0.64,  'GZ: +000.0 °/s', transform=ax_gyro.transAxes,  color='#00E5FF', **_hf)
txt_fft_status = ax_fft.text(0.5, 0.88, f'Filling FFT buffer… (0/{FFT_SIZE})',
                              transform=ax_fft.transAxes, color='#888888',
                              fontsize=9, ha='center', fontfamily='monospace',
                              animated=True)

for a, loc in [(ax_accel, 'upper right'), (ax_gyro, 'upper right'), (ax_fft, 'upper right')]:
    a.legend(loc=loc, facecolor='#000000', edgecolor='#333333',
             prop={'family': 'monospace', 'size': 8})

# ─── FFT COMPUTATION ──────────────────────────────────────────────────────────

def compute_fft():
    """Rectangular-window (no windowing) single-sided magnitude spectrum of AZ buffer."""
    global fft_mag, shaft_f1
    az_arr   = np.array(fft_buf_az, dtype=np.float32)
    az_dc    = az_arr - np.mean(az_arr)                       # remove DC only
    windowed = az_dc * rect_window                            # rectangular = multiply by 1
    spectrum = np.fft.rfft(windowed, n=FFT_SIZE)
    mag      = (2.0 / FFT_SIZE) * np.abs(spectrum)           # single-sided amplitude
    mag[0]   = 0.0                                            # zero the DC bin
    fft_mag  = mag

    # Auto-detect shaft fundamental: highest peak above 5 Hz (ignore very-low-freq drift)
    min_bin = int(5.0 / (SAMPLE_RATE / FFT_SIZE)) + 1        # bin index for 5 Hz
    peak_bin = np.argmax(mag[min_bin:]) + min_bin
    shaft_f1 = fft_freqs[peak_bin]

    # Auto-scale FFT y-axis to 120% of current peak (with a floor of 0.005 g)
    peak_amp = mag[peak_bin]
    new_ymax = max(peak_amp * 1.4, 0.005)
    ax_fft.set_ylim(0, new_ymax)

    return peak_bin, peak_amp


# ─── ANIMATION UPDATE ─────────────────────────────────────────────────────────

def update(frame):
    global latest_ax, latest_ay, latest_az, latest_gx, latest_gy, latest_gz

    # Drain all available serial bytes in one animation frame
    while ser.in_waiting > 0:
        try:
            raw = ser.readline().decode('utf-8', errors='replace').strip()
            # Skip blank lines, comments (#), and CSV header
            if not raw or raw.startswith('#') or raw.startswith('timestamp'):
                continue

            fields = raw.split(',')
            if len(fields) != 7:
                continue

            # timestamp_us, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps
            _ts = int(fields[0])
            latest_ax = float(fields[1])
            latest_ay = float(fields[2])
            latest_az = float(fields[3])
            latest_gx = float(fields[4])
            latest_gy = float(fields[5])
            latest_gz = float(fields[6])

            # Push to time-domain ring buffers
            data_ax.append(latest_ax)
            data_ay.append(latest_ay)
            data_az.append(latest_az)
            data_gx.append(latest_gx)
            data_gy.append(latest_gy)
            data_gz.append(latest_gz)

            # Push AZ to FFT accumulation buffer
            fft_buf_az.append(latest_az)

        except Exception:
            pass   # Silently discard corrupt/partial lines

    # ── Update time-domain lines ──────────────────────────────
    line_ax_t.set_ydata(data_ax)
    line_ay_t.set_ydata(data_ay)
    line_az_t.set_ydata(data_az)
    line_gx_t.set_ydata(data_gx)
    line_gy_t.set_ydata(data_gy)
    line_gz_t.set_ydata(data_gz)

    # ── HUD numeric readouts ──────────────────────────────────
    txt_ax_v.set_text(f"AX: {latest_ax:+.3f} g")
    txt_ay_v.set_text(f"AY: {latest_ay:+.3f} g")
    txt_az_v.set_text(f"AZ: {latest_az:+.3f} g")
    txt_gx_v.set_text(f"GX: {latest_gx:+06.1f} °/s")
    txt_gy_v.set_text(f"GY: {latest_gy:+06.1f} °/s")
    txt_gz_v.set_text(f"GZ: {latest_gz:+06.1f} °/s")

    # ── FFT update (only when 1024-sample buffer is full) ─────
    buf_len = len(fft_buf_az)
    if buf_len < FFT_SIZE:
        txt_fft_status.set_text(f"Filling FFT buffer… ({buf_len}/{FFT_SIZE})")
        peak_marker.set_data([], [])
        peak_text.set_text('')
        for hl in harm_lines:
            hl.set_alpha(0.0)
        for ht in harm_labels:
            ht.set_text('')
    else:
        peak_bin, peak_amp = compute_fft()
        line_fft.set_ydata(fft_mag)

        f1 = shaft_f1
        # Status text: shaft frequency in Hz and equivalent RPM
        rpm_est = f1 * 60.0
        txt_fft_status.set_text(
            f"Shaft f₁ = {f1:.1f} Hz  ({rpm_est:.0f} RPM est.)   "
            f"Peak = {peak_amp*1000:.2f} mg"
        )

        # Peak marker triangle
        peak_marker.set_data([f1], [peak_amp])
        peak_text.set_text(f" {f1:.1f} Hz")
        peak_text.set_position((f1, peak_amp * 1.05))

        # Harmonic lines: 1×, 2×, 3× shaft fundamental
        ymax = ax_fft.get_ylim()[1]
        for k, (hl, ht) in enumerate(zip(harm_lines, harm_labels), start=1):
            hf = f1 * k
            if hf <= SAMPLE_RATE / 2:
                hl.set_xdata([hf, hf])
                hl.set_alpha(0.75)
                ht.set_text(f"{k}×")
                ht.set_position((hf + 1.5, ymax * 0.88))
            else:
                hl.set_alpha(0.0)
                ht.set_text('')

    # Return all animated artists for blit
    return (line_ax_t, line_ay_t, line_az_t,
            line_gx_t, line_gy_t, line_gz_t,
            line_fft,
            txt_ax_v, txt_ay_v, txt_az_v,
            txt_gx_v, txt_gy_v, txt_gz_v,
            txt_fft_status, peak_marker, peak_text,
            *harm_lines, *harm_labels)


# ─── LAUNCH ───────────────────────────────────────────────────────────────────
print("▶  Running — live display only, no CSV saving.")

ani = animation.FuncAnimation(
    fig, update, interval=FPS_INTERVAL, blit=True, cache_frame_data=False
)

plt.show()

ser.close()
print("System Offline.")
