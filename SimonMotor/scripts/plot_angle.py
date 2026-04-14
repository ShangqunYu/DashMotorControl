#!/usr/bin/env python3
"""Live plot motor d/q and phase currents from the STM32 UART stream."""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


CSV_HEADER = ("iq", "i_q_des", "i_q_filt", "id", "id_des", "id_filt", "i_a", "i_b", "i_c", "m_angle", "e_angle", "rpm", "rpm_ref")
LABELED_PATTERNS = {
    "iq": re.compile(r"^iq:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "i_q_des": re.compile(r"^i_q_des:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "i_q_filt": re.compile(r"^i_q_filt:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "id": re.compile(r"^id:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "id_des": re.compile(r"^id_des:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "id_filt": re.compile(r"^id_filt:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "i_a": re.compile(r"^i_a:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "i_b": re.compile(r"^i_b:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "i_c": re.compile(r"^i_c:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "m_angle": re.compile(r"^m_angle:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "e_angle": re.compile(r"^e_angle:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "rpm": re.compile(r"^rpm:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
    "rpm_ref": re.compile(r"^rpm_ref:\s*(-?\d+(?:\.\d+)?)$", re.IGNORECASE),
}


class CurrentPlotter:
    def __init__(self, port: str, baudrate: int, window_seconds: float) -> None:
        self.port = port
        self.baudrate = baudrate
        self.window_seconds = window_seconds
        self.time_data: deque[float] = deque()
        self.series = {name: deque() for name in CSV_HEADER}
        self.pending_labeled: dict[str, float] = {}
        self.start_time = time.monotonic()
        self.running = True
        self.lock = threading.Lock()

    def append_sample(self, sample: dict[str, float]) -> None:
        now = time.monotonic() - self.start_time
        with self.lock:
            self.time_data.append(now)
            for name in CSV_HEADER:
                self.series[name].append(sample[name])

            cutoff = now - self.window_seconds
            while self.time_data and self.time_data[0] < cutoff:
                self.time_data.popleft()
                for name in CSV_HEADER:
                    self.series[name].popleft()

    def try_parse_csv(self, line: str) -> dict[str, float] | None:
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != len(CSV_HEADER):
            return None
        if tuple(parts) == CSV_HEADER:
            return None
        try:
            values = [float(part) for part in parts]
        except ValueError:
            return None
        return dict(zip(CSV_HEADER, values))

    def try_parse_labeled(self, line: str) -> dict[str, float] | None:
        for name, pattern in LABELED_PATTERNS.items():
            match = pattern.match(line)
            if match is None:
                continue
            self.pending_labeled[name] = float(match.group(1))
            if all(key in self.pending_labeled for key in CSV_HEADER):
                sample = {key: self.pending_labeled[key] for key in CSV_HEADER}
                self.pending_labeled.clear()
                return sample
            return None
        return None

    def serial_worker(self) -> None:
        try:
            with serial.Serial(self.port, self.baudrate, timeout=1.0) as ser:
                while self.running:
                    raw = ser.readline()
                    if not raw:
                        continue

                    try:
                        line = raw.decode("utf-8", errors="ignore").strip()
                    except UnicodeDecodeError:
                        continue

                    if not line:
                        continue

                    sample = self.try_parse_csv(line)
                    if sample is None:
                        sample = self.try_parse_labeled(line)

                    if sample is not None:
                        self.append_sample(sample)
        except serial.SerialException as exc:
            print(f"Serial error on {self.port}: {exc}", file=sys.stderr)
            self.running = False

    def run(self) -> None:
        worker = threading.Thread(target=self.serial_worker, daemon=True)
        worker.start()

        fig_iq, ax_iq = plt.subplots()
        fig_id, ax_id = plt.subplots()
        fig_phase, ax_phase = plt.subplots()
        fig_angle, ax_angle = plt.subplots()
        fig_rpm, ax_rpm = plt.subplots()

        iq_lines = {
            "iq": ax_iq.plot([], [], label="iq", linewidth=2.0)[0],
            "i_q_des": ax_iq.plot([], [], label="i_q_des", linewidth=2.0, linestyle="--")[0],
            "i_q_filt": ax_iq.plot([], [], label="i_q_filt", linewidth=2.0, linestyle=":")[0],
        }
        id_lines = {
            "id": ax_id.plot([], [], label="id", linewidth=2.0)[0],
            "id_des": ax_id.plot([], [], label="id_des", linewidth=2.0, linestyle="--")[0],
            "id_filt": ax_id.plot([], [], label="id_filt", linewidth=2.0, linestyle=":")[0],
        }
        phase_lines = {
            "i_a": ax_phase.plot([], [], label="i_a", linewidth=2.0)[0],
            "i_b": ax_phase.plot([], [], label="i_b", linewidth=2.0, linestyle="--")[0],
            "i_c": ax_phase.plot([], [], label="i_c", linewidth=2.0, linestyle=":")[0],
        }
        angle_lines = {
            "m_angle": ax_angle.plot([], [], label="m_angle", linewidth=2.0)[0],
            "e_angle": ax_angle.plot([], [], label="e_angle", linewidth=2.0, linestyle="--")[0],
        }
        rpm_lines = {
            "rpm": ax_rpm.plot([], [], label="rpm", linewidth=2.0)[0],
            "rpm_ref": ax_rpm.plot([], [], label="rpm_ref", linewidth=2.0, linestyle="--")[0],
        }

        ax_iq.set_title(f"q-axis Current: {self.port} @ {self.baudrate}")
        ax_iq.set_xlabel("Time (s)")
        ax_iq.set_ylabel("Current (A)")
        ax_iq.grid(True, alpha=0.3)
        ax_iq.legend(loc="upper right")

        ax_id.set_title(f"d-axis Current: {self.port} @ {self.baudrate}")
        ax_id.set_xlabel("Time (s)")
        ax_id.set_ylabel("Current (A)")
        ax_id.grid(True, alpha=0.3)
        ax_id.legend(loc="upper right")

        ax_phase.set_title(f"Phase Currents: {self.port} @ {self.baudrate}")
        ax_phase.set_xlabel("Time (s)")
        ax_phase.set_ylabel("Current (A)")
        ax_phase.grid(True, alpha=0.3)
        ax_phase.legend(loc="upper right")

        ax_angle.set_title(f"Motor Angles: {self.port} @ {self.baudrate}")
        ax_angle.set_xlabel("Time (s)")
        ax_angle.set_ylabel("Angle (rad)")
        ax_angle.grid(True, alpha=0.3)
        ax_angle.legend(loc="upper right")

        ax_rpm.set_title(f"Motor RPM: {self.port} @ {self.baudrate}")
        ax_rpm.set_xlabel("Time (s)")
        ax_rpm.set_ylabel("RPM")
        ax_rpm.grid(True, alpha=0.3)
        ax_rpm.legend(loc="upper right")

        def update(_frame: int):
            with self.lock:
                x = list(self.time_data)
                data = {name: list(values) for name, values in self.series.items()}

            if not x:
                return tuple(iq_lines.values()) + tuple(id_lines.values()) + tuple(phase_lines.values()) + tuple(angle_lines.values())

            for name in iq_lines:
                iq_lines[name].set_data(x, data[name])
            for name in id_lines:
                id_lines[name].set_data(x, data[name])
            for name in phase_lines:
                phase_lines[name].set_data(x, data[name])
            for name in angle_lines:
                angle_lines[name].set_data(x, data[name])
            for name in rpm_lines:
                rpm_lines[name].set_data(x, data[name])

            x_min = max(0.0, x[-1] - self.window_seconds)
            x_max = max(self.window_seconds, x[-1])
            ax_iq.set_xlim(x_min, x_max)
            ax_id.set_xlim(x_min, x_max)
            ax_phase.set_xlim(x_min, x_max)
            ax_angle.set_xlim(x_min, x_max)
            ax_rpm.set_xlim(x_min, x_max)

            iq_values = data["iq"] + data["i_q_des"] + data["i_q_filt"]
            id_values = data["id"] + data["id_des"] + data["id_filt"]

            iq_min = min(iq_values)
            iq_max = max(iq_values)
            iq_margin = max(0.5, 0.1 * max(abs(iq_min), abs(iq_max), 1.0))
            ax_iq.set_ylim(iq_min - iq_margin, iq_max + iq_margin)

            id_min = min(id_values)
            id_max = max(id_values)
            id_margin = max(0.5, 0.1 * max(abs(id_min), abs(id_max), 1.0))
            ax_id.set_ylim(id_min - id_margin, id_max + id_margin)

            phase_values = data["i_a"] + data["i_b"] + data["i_c"]
            phase_min = min(phase_values)
            phase_max = max(phase_values)
            phase_margin = max(0.5, 0.1 * max(abs(phase_min), abs(phase_max), 1.0))
            ax_phase.set_ylim(phase_min - phase_margin, phase_max + phase_margin)

            angle_values = data["m_angle"] + data["e_angle"]
            angle_min = min(angle_values)
            angle_max = max(angle_values)
            angle_margin = max(0.1, 0.1 * max(abs(angle_min), abs(angle_max), 1.0))
            ax_angle.set_ylim(angle_min - angle_margin, angle_max + angle_margin)

            rpm_values = data["rpm"] + data["rpm_ref"]
            rpm_min = min(rpm_values)
            rpm_max = max(rpm_values)
            rpm_margin = max(10.0, 0.1 * max(abs(rpm_min), abs(rpm_max), 1.0))
            ax_rpm.set_ylim(rpm_min - rpm_margin, rpm_max + rpm_margin) 

            return tuple(iq_lines.values()) + tuple(id_lines.values()) + tuple(phase_lines.values()) + tuple(angle_lines.values()) + tuple(rpm_lines.values())

        animation_iq = FuncAnimation(fig_iq, update, interval=50, blit=False, cache_frame_data=False)
        animation_id = FuncAnimation(fig_id, update, interval=50, blit=False, cache_frame_data=False)
        animation_phase = FuncAnimation(fig_phase, update, interval=50, blit=False, cache_frame_data=False)
        animation_angle = FuncAnimation(fig_angle, update, interval=50, blit=False, cache_frame_data=False)
        animation_rpm = FuncAnimation(fig_rpm, update, interval=50, blit=False, cache_frame_data=False)
        try:
            plt.show()
        finally:
            self.running = False
            _ = animation_iq
            _ = animation_id
            _ = animation_phase
            _ = animation_angle
            _ = animation_rpm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live plot iq/id and phase current telemetry over UART.")
    parser.add_argument("port", help="Serial port, for example /dev/ttyUSB0 or COM3")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--window", type=float, default=10.0, help="Visible time window in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    plotter = CurrentPlotter(args.port, args.baud, args.window)
    plotter.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
