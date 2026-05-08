#!/usr/bin/env python3
"""
Live current graph for ACS712 / ESP32.
Usage:
    python graph.py             # auto-detect COM port
    python graph.py --port COM3
Requirements: pip install pyserial matplotlib
"""

import argparse
import collections
import threading

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports

# ── settings ────────────────────────────────────────────────────────────────
BAUD        = 921600
SAMPLE_HZ   = 1000          # must match SAMPLE_INTERVAL_US=1000 in sketch
WINDOW_MS   = 5000          # visible rolling window (ms)
WINDOW_PTS  = SAMPLE_HZ * WINDOW_MS // 1000   # = 5000 points

LPF_ALPHA   = 0.05          # Low-pass filter smoothing factor (0.01 to 1.0)
Y_LIM_MIN   = -2.0          # Fixed Y-axis minimum (Amps)
Y_LIM_MAX   = 2.0           # Fixed Y-axis maximum (Amps)

# ── shared buffers (filled by serial thread, read by animation) ─────────────
buf: collections.deque[tuple[float, float]] = collections.deque(maxlen=WINDOW_PTS)
buf_lock  = threading.Lock()
start_ms: list[float] = []
last_filtered: list[float] = []


def serial_thread(port: str) -> None:
    """Background thread: read lines from ESP32, apply LPF, and push data."""
    import time as _time
    with serial.Serial(port, BAUD, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode("ascii", errors="ignore").strip()
            except serial.SerialException:
                break
            
            if not line or line.startswith("#") or "timestamp" in line.lower():
                continue
            
            parts = line.split(",")
            if len(parts) < 2:
                continue
            
            try:
                raw_current = float(parts[1])
            except ValueError:
                continue

            # Apply Exponential Moving Average (EMA) Low-Pass Filter
            if not last_filtered:
                last_filtered.append(raw_current)
            else:
                last_filtered[0] = (LPF_ALPHA * raw_current) + ((1.0 - LPF_ALPHA) * last_filtered[0])
            
            filtered_current = last_filtered[0]
            now_ms = _time.monotonic() * 1000.0
            
            with buf_lock:
                if not start_ms:
                    start_ms.append(now_ms)
                elapsed = now_ms - start_ms[0]
                buf.append((elapsed, filtered_current))


def autodetect_port() -> str:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").upper()
        if any(k in desc for k in ("CP210", "CH340", "FTDI", "USB")):
            return p.device
    ports = serial.tools.list_ports.comports()
    if ports:
        return ports[0].device
    raise RuntimeError("No serial port found. Use --port COMx.")


def main() -> None:
    parser = argparse.ArgumentParser(description="ACS712 live current graph")
    parser.add_argument("--port", default=None, help="Serial port (e.g. COM3)")
    args = parser.parse_args()

    port = args.port or autodetect_port()
    print(f"[INFO] Opening {port} at {BAUD} baud…")

    t = threading.Thread(target=serial_thread, args=(port,), daemon=True)
    t.start()

    # ── figure ────────────────────────────────────────────────────────────
    fig, ax = plt.subplots()
    ax.set_title("ACS712 — Motor Current (Live & Filtered)")
    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Current [A]")
    
    # Apply fixed Y-axis limits
    ax.set_ylim(Y_LIM_MIN, Y_LIM_MAX)
    ax.grid(True)

    (line,) = ax.plot([], [], linewidth=1)

    def update(_frame):
        with buf_lock:
            pairs = list(buf)
        if not pairs:
            return (line,)
        
        xs = [p[0] for p in pairs]
        ys = [p[1] for p in pairs]
        line.set_data(xs, ys)
        
        # scroll X window: show the last WINDOW_MS milliseconds
        x_end   = xs[-1]
        x_start = max(0.0, x_end - WINDOW_MS)
        ax.set_xlim(x_start, x_end)
        
        return (line,)

    _anim = animation.FuncAnimation(
        fig, update, interval=50, blit=False, cache_frame_data=False
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()