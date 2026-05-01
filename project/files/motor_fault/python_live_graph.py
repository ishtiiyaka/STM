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
MAX_POINTS = 80             
SPLINE_POINTS = 300         
VIBE_LIMITS = (-5, 5)       # Accel in 'g' units from Arduino
CURR_LIMITS = (0, 5)        # 0A to 5A range
TEMP_LIMITS = (20, 60)      # Normal operating temp range
FPS_INTERVAL = 16           

# --- FILTERS ---
ALPHA_VIBE = 0.2
ALPHA_CURR = 0.1

# Serial connection
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"🛰  Telemetry Linked: {COM_PORT} at {BAUD_RATE} baud")
except Exception as e:
    print(f"❌ Serial error: {e}")
    exit()

# Data containers
data_ax = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
data_ay = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
data_az = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
data_c1 = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
data_c2 = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# State variables
curr_ax, curr_ay, curr_az = 0.0, 0.0, 0.0
curr_c1, curr_c2, curr_temp = 0.0, 0.0, 0.0

x_axis = np.arange(MAX_POINTS)
x_axis_smooth = np.linspace(0, MAX_POINTS - 1, SPLINE_POINTS)

# ------------------------------------------------------------
#  UI LAYOUT & STYLING
# ------------------------------------------------------------
plt.style.use('dark_background')
fig = plt.figure(figsize=(14, 8), facecolor='#030308')
gs = fig.add_gridspec(2, 3)

# Panel 1: Main Vibration (MPU6050)
ax_vibe = fig.add_subplot(gs[:, :2], facecolor='#050510')
ax_vibe.set_title("VIBRATION TELEMETRY (g)", color='#00e5ff', fontweight='bold')
ax_vibe.set_ylim(*VIBE_LIMITS)
ax_vibe.grid(True, color='#00e5ff', alpha=0.05)

# Panel 2: Motor Currents
ax_curr = fig.add_subplot(gs[0, 2], facecolor='#050510')
ax_curr.set_title("MOTOR LOAD (A)", color='#39ff14', fontweight='bold')
ax_curr.set_ylim(*CURR_LIMITS)

# Panel 3: Temperature
ax_temp = fig.add_subplot(gs[1, 2], facecolor='#050510')
ax_temp.set_title("THERMAL SENSOR (°C)", color='#ff7b3d', fontweight='bold')
ax_temp.set_ylim(*TEMP_LIMITS)

# Styling clean-up
for a in [ax_vibe, ax_curr, ax_temp]:
    for spine in a.spines.values(): spine.set_visible(False)
    a.tick_params(colors='#444466', labelsize=8)

# --- HUD ELEMENTS ---
def make_neon_line(target_ax, color, label):
    l, = target_ax.plot(x_axis_smooth, [0]*SPLINE_POINTS, color=color, 
                        linewidth=2, label=label, solid_capstyle='round', animated=True)
    l.set_path_effects([patheffects.withStroke(linewidth=4, foreground=color, alpha=0.2)])
    return l

line_ax = make_neon_line(ax_vibe, '#ff3d57', 'AX')
line_ay = make_neon_line(ax_vibe, '#39ff14', 'AY')
line_az = make_neon_line(ax_vibe, '#00e5ff', 'AZ')

line_c1 = ax_curr.plot(x_axis, [0]*MAX_POINTS, color='#39ff14', label='M1 (Healthy)', animated=True)[0]
line_c2 = ax_curr.plot(x_axis, [0]*MAX_POINTS, color='#ff3d57', label='M2 (Test)', animated=True)[0]

# Static labels for current values
txt_c1 = ax_curr.text(0.05, 0.9, '', transform=ax_curr.transAxes, color='#39ff14', fontweight='bold')
txt_c2 = ax_curr.text(0.05, 0.8, '', transform=ax_curr.transAxes, color='#ff3d57', fontweight='bold')
txt_temp = ax_temp.text(0.5, 0.5, '-- °C', transform=ax_temp.transAxes, 
                        fontsize=30, ha='center', va='center', color='#ff7b3d', fontweight='bold')

ax_vibe.legend(loc='upper right', fontsize=8)
ax_curr.legend(loc='upper right', fontsize=7)

# --------------------- PROCESSING ---------------------

def get_smooth(data):
    try:
        return make_interp_spline(x_axis, data, k=3)(x_axis_smooth)
    except: return np.zeros(SPLINE_POINTS)

def update(frame):
    global curr_ax, curr_ay, curr_az, curr_c1, curr_c2, curr_temp

    while ser.in_waiting > 0:
        try:
            # Expected: timestamp, curr1, curr2, temp, ax, ay, az, gx, gy, gz, fs
            raw = ser.readline().decode('utf-8').strip().split(',')
            if len(raw) == 11:
                # 1. Currents (A)
                c1_val = float(raw[1])
                c2_val = float(raw[2])
                curr_c1 = (ALPHA_CURR * c1_val) + (1 - ALPHA_CURR) * curr_c1
                curr_c2 = (ALPHA_CURR * c2_val) + (1 - ALPHA_CURR) * curr_c2
                data_c1.append(curr_c1)
                data_c2.append(curr_c2)

                # 2. Temperature (C)
                curr_temp = float(raw[3])

                # 3. MPU6050 (g)
                rax, ray, raz = float(raw[4]), float(raw[5]), float(raw[6])
                curr_ax = (ALPHA_VIBE * rax) + (1 - ALPHA_VIBE) * curr_ax
                curr_ay = (ALPHA_VIBE * ray) + (1 - ALPHA_VIBE) * curr_ay
                curr_az = (ALPHA_VIBE * raz) + (1 - ALPHA_VIBE) * curr_az
                data_ax.append(curr_ax)
                data_ay.append(curr_ay)
                data_az.append(curr_az)
        except: pass

    # Update Vibration Lines (Splined)
    line_ax.set_ydata(get_smooth(data_ax))
    line_ay.set_ydata(get_smooth(data_ay))
    line_az.set_ydata(get_smooth(data_az))

    # Update Current Lines (Standard plot for speed)
    line_c1.set_ydata(data_c1)
    line_c2.set_ydata(data_c2)

    # Update Text Stats
    txt_c1.set_text(f"M1: {curr_c1:.2f} A")
    txt_c2.set_text(f"M2: {curr_c2:.2f} A")
    txt_temp.set_text(f"{curr_temp:.1f} °C")

    return line_ax, line_ay, line_az, line_c1, line_c2, txt_c1, txt_c2, txt_temp

# --------------------- LAUNCH ---------------------
ani = animation.FuncAnimation(fig, update, interval=FPS_INTERVAL, blit=True, cache_frame_data=False)

plt.tight_layout()
plt.show()

ser.close()
print("System Offline.")