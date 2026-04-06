#!/usr/bin/env python3
import argparse
import math
import re
import sys
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


ANGLE_PATTERN = re.compile(r"Angle:\s*([-+]?\d+(?:\.\d+)?)")
RPM_PATTERN = re.compile(r"RPM:\s*([-+]?\d+(?:\.\d+)?)")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read angle and RPM values from serial and plot them live."
    )
    parser.add_argument("port", help="Serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. Default: 115200",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=300,
        help="Number of points to keep in the live plot. Default: 300",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Could not open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    angle_times = deque(maxlen=args.window)
    angles = deque(maxlen=args.window)
    rpm_times = deque(maxlen=args.window)
    rpms = deque(maxlen=args.window)
    rad_s_times = deque(maxlen=args.window)
    rad_s_values = deque(maxlen=args.window)
    start_time = time.time()

    fig, (ax_angle, ax_rpm, ax_rad_s) = plt.subplots(3, 1, sharex=True)
    angle_line, = ax_angle.plot([], [], lw=2)
    rpm_line, = ax_rpm.plot([], [], lw=2, color="tab:red")
    rad_s_line, = ax_rad_s.plot([], [], lw=2, color="tab:green")

    ax_angle.set_title("Motor Angle")
    ax_angle.set_ylabel("Angle (deg)")
    ax_angle.grid(True)

    ax_rpm.set_title("Motor Speed")
    ax_rpm.set_ylabel("RPM")
    ax_rpm.grid(True)

    ax_rad_s.set_title("Motor Speed")
    ax_rad_s.set_xlabel("Time (s)")
    ax_rad_s.set_ylabel("rad/s")
    ax_rad_s.grid(True)

    def update(_frame):
        while ser.in_waiting:
            raw_line = ser.readline().decode("utf-8", errors="ignore").strip()
            elapsed = time.time() - start_time
            angle_match = ANGLE_PATTERN.search(raw_line)
            if angle_match:
                angle_times.append(elapsed)
                angles.append(float(angle_match.group(1)))
                continue

            rpm_match = RPM_PATTERN.search(raw_line)
            if rpm_match:
                rpm_value = float(rpm_match.group(1))
                rpm_times.append(elapsed)
                rpms.append(rpm_value)
                rad_s_times.append(elapsed)
                rad_s_values.append(rpm_value * 2.0 * math.pi / 60.0)

        if not angle_times and not rpm_times:
            return (angle_line, rpm_line, rad_s_line)

        if angle_times:
            angle_line.set_data(angle_times, angles)
            ax_angle.set_xlim(angle_times[0], max(angle_times[-1], angle_times[0] + 1e-3))

            angle_min = min(angles)
            angle_max = max(angles)
            if angle_min == angle_max:
                angle_pad = 1.0
            else:
                angle_pad = max((angle_max - angle_min) * 0.1, 1.0)
            ax_angle.set_ylim(angle_min - angle_pad, angle_max + angle_pad)

        if rpm_times:
            rpm_line.set_data(rpm_times, rpms)

            rpm_min = min(rpms)
            rpm_max = max(rpms)
            if rpm_min == rpm_max:
                rpm_pad = 1.0
            else:
                rpm_pad = max((rpm_max - rpm_min) * 0.1, 1.0)
            ax_rpm.set_ylim(rpm_min - rpm_pad, rpm_max + rpm_pad)

        if rad_s_times:
            rad_s_line.set_data(rad_s_times, rad_s_values)

            rad_s_min = min(rad_s_values)
            rad_s_max = max(rad_s_values)
            if rad_s_min == rad_s_max:
                rad_s_pad = 1.0
            else:
                rad_s_pad = max((rad_s_max - rad_s_min) * 0.1, 1.0)
            ax_rad_s.set_ylim(rad_s_min - rad_s_pad, rad_s_max + rad_s_pad)

        x_start_candidates = []
        x_end_candidates = []
        if angle_times:
            x_start_candidates.append(angle_times[0])
            x_end_candidates.append(angle_times[-1])
        if rpm_times:
            x_start_candidates.append(rpm_times[0])
            x_end_candidates.append(rpm_times[-1])
        if rad_s_times:
            x_start_candidates.append(rad_s_times[0])
            x_end_candidates.append(rad_s_times[-1])

        if x_start_candidates and x_end_candidates:
            x_start = min(x_start_candidates)
            x_end = max(x_end_candidates)
            ax_rad_s.set_xlim(x_start, max(x_end, x_start + 1e-3))

        return (angle_line, rpm_line, rad_s_line)

    def on_close(_event):
        ser.close()

    fig.canvas.mpl_connect("close_event", on_close)
    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()
    del anim
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
