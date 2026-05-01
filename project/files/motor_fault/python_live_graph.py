import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patheffects
from collections import deque
import numpy as np
from scipy.interpolate import make_interp_spline

# --- CONFIGURATION ---
COM_PORT = 'COM5'          
BAUD_RATE = 115200
MAX_POINTS = 80             # Slightly fewer base points for cleaner splines
SPLINE_POINTS = 300         # The high-resolution curve points (Visual Smoothness)
Y_LIMITS = (-20000, 20000)  
FPS_INTERVAL = 16           # ~60 FPS update rate

# --- EMA FILTER SETTINGS ---
# 0.01 = Extremely smooth but delayed | 1.0 = Instant but jagged
ALPHA = 0.15  

# Serial connection
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"🛰  Connected to {COM_PORT} – live telemetry active")
except Exception as e:
    print(f"❌ Serial error: {e}")
    exit()

# Data containers
data_x = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
data_y = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
data_z = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# State variables for EMA Filter
curr_x, curr_y, curr_z = 0.0, 0.0, 0.0

# X-axis arrays for Spline interpolation
x_axis = np.arange(MAX_POINTS)
x_axis_smooth = np.linspace(0, MAX_POINTS - 1, SPLINE_POINTS)

# ------------------------------------------------------------
#  GRAPH STYLING
# ------------------------------------------------------------
plt.style.use('dark_background')
fig = plt.figure(figsize=(10, 6), facecolor='#050510')
ax = fig.add_subplot(111, facecolor='#0a0a12')
fig.canvas.manager.set_window_title('MPU6050 • ULTRA SMOOTH HUD')

for spine in ax.spines.values():
    spine.set_visible(False)
ax.tick_params(colors='#2a2a3a', labelsize=8)
ax.set_xlim(0, MAX_POINTS - 1)
ax.set_ylim(*Y_LIMITS)
ax.grid(True, color='#00e5ff', alpha=0.08, linewidth=0.6)

title = ax.set_title(
    "LIVE  MPU6050  TELEMETRY",
    fontsize=18, fontweight='900', color='#00e5ff',
    family='monospace', loc='center',
    path_effects=[patheffects.withStroke(linewidth=4, foreground='#003344')]
)

# --- NEON LINES ---
def make_glowlines(color, linewidth_main=2.5):
    glow_layers = [(8, 0.05), (5, 0.15), (3, 0.35), (linewidth_main, 1.0)]
    lines = []
    for lw, alpha in glow_layers:
        # Initialize with high-resolution x_axis_smooth
        l, = ax.plot(x_axis_smooth, [0]*SPLINE_POINTS, color=color, alpha=alpha,
                     linewidth=lw, solid_capstyle='round', animated=True)
        lines.append(l)
    return lines

lines_x = make_glowlines('#ff3d57')
lines_y = make_glowlines('#39ff14')
lines_z = make_glowlines('#00e5ff')

# --- SCANNING LINE ---
scan_line = ax.axhline(0, color='#00e5ff', alpha=0.25, linewidth=1.5, linestyle='-', animated=True)

# --- END-POINT DOTS ---
latest_idx = MAX_POINTS - 1
scat_x = ax.scatter([latest_idx], [0], color='#ff3d57', edgecolors='white', linewidth=1, s=100, zorder=10, animated=True)
scat_y = ax.scatter([latest_idx], [0], color='#39ff14', edgecolors='white', linewidth=1, s=100, zorder=10, animated=True)
scat_z = ax.scatter([latest_idx], [0], color='#00e5ff', edgecolors='white', linewidth=1, s=100, zorder=10, animated=True)

legend = ax.legend(
    handles=[lines_x[-1], lines_y[-1], lines_z[-1]],
    labels=['X', 'Y', 'Z'], loc='upper left', facecolor='#0a0a12',
    edgecolor='#1a1a2e', labelcolor='#cccccc', prop={'size': 10, 'weight': 'bold'}
)

# --------------------- ANIMATION LOOP ---------------------
scan_y = 0
scan_dir = 1
scan_speed = 600

def get_smooth_curve(data):
    """Converts rough points into a smooth fluid curve using Splines"""
    try:
        spline = make_interp_spline(x_axis, data, k=3) # k=3 is cubic spline
        return spline(x_axis_smooth)
    except:
        return np.zeros(SPLINE_POINTS)

def update(frame):
    global scan_y, scan_dir, curr_x, curr_y, curr_z

    # Read serial data
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            parts = line.split(',')
            if len(parts) == 3:
                raw_x = int(parts[0].split(':')[1])
                raw_y = int(parts[1].split(':')[1])
                raw_z = int(parts[2].split(':')[1])
                
                # Apply EMA (Exponential Moving Average) Filter
                curr_x = (ALPHA * raw_x) + ((1 - ALPHA) * curr_x)
                curr_y = (ALPHA * raw_y) + ((1 - ALPHA) * curr_y)
                curr_z = (ALPHA * raw_z) + ((1 - ALPHA) * curr_z)

                data_x.append(curr_x)
                data_y.append(curr_y)
                data_z.append(curr_z)
        except Exception:
            pass

    # Generate Splines
    smooth_x = get_smooth_curve(data_x)
    smooth_y = get_smooth_curve(data_y)
    smooth_z = get_smooth_curve(data_z)

    # Update glow layers
    for line_set, smooth_data in [(lines_x, smooth_x), (lines_y, smooth_y), (lines_z, smooth_z)]:
        for l in line_set:
            l.set_ydata(smooth_data)

    # Move scanning line smoothly
    scan_y += scan_dir * scan_speed * (FPS_INTERVAL / 1000.0)
    if scan_y > Y_LIMITS[1]:
        scan_y, scan_dir = Y_LIMITS[1], -1
    elif scan_y < Y_LIMITS[0]:
        scan_y, scan_dir = Y_LIMITS[0], 1
    scan_line.set_ydata([scan_y, scan_y])

    # Update dots
    scat_x.set_offsets([[latest_idx, smooth_x[-1]]])
    scat_y.set_offsets([[latest_idx, smooth_y[-1]]])
    scat_z.set_offsets([[latest_idx, smooth_z[-1]]])

    return (
        [item for sublist in [lines_x, lines_y, lines_z] for item in sublist] +
        [scan_line, scat_x, scat_y, scat_z]
    )

# --------------------- LAUNCH DASHBOARD ---------------------
ani = animation.FuncAnimation(fig, update, interval=FPS_INTERVAL, blit=True, cache_frame_data=False)

plt.tight_layout(pad=2)
plt.show()

ser.close()
print("Dashboard closed.")