"""
MIT-mode sine trajectory test.

Switches the motor to MIT mode, then streams position commands following a
sine wave.  TX and RX run in separate threads so the send rate is never
blocked waiting for a reply.  Press Ctrl+C or the Stop button to stop —
the motor is returned to MENU_MODE before the script exits.

Usage:
    python3 mit_sine.py [channel] [can_id]

Examples:
    python3 mit_sine.py              # can0, id=1
    python3 mit_sine.py can0 1
    python3 mit_sine.py can1 3
"""

import sys
import math
import time
import threading
import queue
import collections
import can
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

CHANNEL = sys.argv[1] if len(sys.argv) > 1 else "can0"
CAN_ID  = int(sys.argv[2]) if len(sys.argv) > 2 else 1

# ── Mode bytes (must match foc.h) ────────────────────────────────────────────
MENU_MODE = 0
MIT_MODE  = 5

# ── Scaling ranges — must match flash-stored values on the motor ─────────────
P_MIN   = -12.5     # rad
P_MAX   =  12.5     # rad
V_MIN   = -45.0     # rad/s
V_MAX   =  45.0     # rad/s
KP_MAX  =  500.0    # N-m/rad
KD_MAX  =  5.0      # N-m·s/rad
T_MIN   = -18.0     # N-m
T_MAX   =  18.0     # N-m

# ── Sine trajectory parameters ────────────────────────────────────────────────
AMPLITUDE   = 3.14  # rad   (peak displacement from zero)
FREQUENCY   = 2  # Hz
UPDATE_HZ   = 200   # command rate
CENTER      = 0.0   # rad   (center of sine wave)

# ── Control gains ─────────────────────────────────────────────────────────────
KP = 20.0   # N-m/rad
KD = 0.5    # N-m·s/rad

# ── Plot settings ─────────────────────────────────────────────────────────────
PLOT_WINDOW_S    = 5   # seconds of history shown
PLOT_UPDATE_EVERY = 5   # redraw every N received frames


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))

def uint_to_float(x, x_min, x_max, bits):
    return x * (x_max - x_min) / ((1 << bits) - 1) + x_min


def pack_cmd(p, v, kp, kd, t_ff):
    p_int  = float_to_uint(p,    P_MIN,  P_MAX,  16)
    v_int  = float_to_uint(v,    V_MIN,  V_MAX,  12)
    kp_int = float_to_uint(kp,   0,      KP_MAX, 12)
    kd_int = float_to_uint(kd,   0,      KD_MAX, 12)
    t_int  = float_to_uint(t_ff, T_MIN,  T_MAX,  12)
    return [
        p_int >> 8,
        p_int & 0xFF,
        v_int >> 4,
        ((v_int & 0xF) << 4) | (kp_int >> 8),
        kp_int & 0xFF,
        kd_int >> 4,
        ((kd_int & 0xF) << 4) | (t_int >> 8),
        t_int & 0xFF,
    ]


def send_mode(bus, mode):
    bus.send(can.Message(
        arbitration_id=CAN_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode],
        is_extended_id=True,
    ), timeout=0.2)


def decode_reply(msg):
    d = msg.data
    if len(d) < 8:
        return None
    p_int = (d[1] << 8) | d[2]
    v_int = (d[3] << 4) | (d[4] >> 4)
    t_int = ((d[4] & 0xF) << 8) | d[5]
    return (
        uint_to_float(p_int, P_MIN, P_MAX, 16),
        uint_to_float(v_int, V_MIN, V_MAX, 12),
        uint_to_float(t_int, T_MIN, T_MAX, 12),
        uint_to_float(d[6],  0.0,  60.0,   8),
        uint_to_float(d[7], -40.0, 125.0,  8),
    )


class LivePlot:
    def __init__(self):
        maxlen = int(PLOT_WINDOW_S * UPDATE_HZ)
        self._t   = collections.deque(maxlen=maxlen)
        self._des = collections.deque(maxlen=maxlen)
        self._act = collections.deque(maxlen=maxlen)
        self._vel = collections.deque(maxlen=maxlen)
        self._tor = collections.deque(maxlen=maxlen)
        self._dt  = collections.deque(maxlen=maxlen)
        self._counter = 0
        self.running = True

        plt.ion()
        self._fig, (self._ax, self._ax_vel, self._ax_tor, self._ax_dt) = \
            plt.subplots(4, 1, sharex=True)
        self._fig.subplots_adjust(bottom=0.12, hspace=0.45)

        self._line_des, = self._ax.plot([], [], label="desired", color="tab:blue",   marker=".", markersize=4)
        self._line_act, = self._ax.plot([], [], label="actual",  color="tab:orange", marker=".", markersize=4)
        self._ax.set_ylabel("position (rad)")
        self._ax.legend()
        self._ax.grid(True)

        self._line_vel, = self._ax_vel.plot([], [], color="tab:purple", marker=".", markersize=4)
        self._ax_vel.set_ylabel("velocity (rad/s)")
        self._ax_vel.grid(True)

        self._line_tor, = self._ax_tor.plot([], [], color="tab:red", marker=".", markersize=4)
        self._ax_tor.set_ylabel("torque (N·m)")
        self._ax_tor.grid(True)

        self._line_dt, = self._ax_dt.plot([], [], color="tab:green", marker=".", markersize=4)
        self._ax_dt.axhline(1.0 / UPDATE_HZ, color="gray", linestyle="--", linewidth=1,
                            label=f"target ({1000/UPDATE_HZ:.1f} ms)")
        self._ax_dt.set_xlabel("time (s)")
        self._ax_dt.set_ylabel("dt (s)")
        self._ax_dt.legend()
        self._ax_dt.grid(True)

        ax_btn = self._fig.add_axes([0.4, 0.02, 0.2, 0.04])
        self._btn = Button(ax_btn, "Stop Motor")
        self._btn.on_clicked(lambda _: setattr(self, "running", False))

    def update(self, t, des, pos, vel, torque):
        dt = t - self._t[-1] if self._t else 0.0
        self._t.append(t);   self._des.append(des); self._act.append(pos)
        self._vel.append(vel); self._tor.append(torque); self._dt.append(dt)
        self._counter += 1
        if self._counter % PLOT_UPDATE_EVERY == 0:
            self._line_des.set_data(self._t, self._des)
            self._line_act.set_data(self._t, self._act)
            self._line_vel.set_data(self._t, self._vel)
            self._line_tor.set_data(self._t, self._tor)
            self._line_dt.set_data(self._t, self._dt)
            for ax in (self._ax, self._ax_vel, self._ax_tor, self._ax_dt):
                ax.relim(); ax.autoscale_view()
            self._fig.canvas.flush_events()

    def close(self):
        plt.ioff()
        plt.show(block=True)


def main():
    print(f"Opening {CHANNEL} ...")
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as exc:
        sys.exit(f"Failed to open {CHANNEL}: {exc}")

    stop     = threading.Event()
    rx_queue = queue.Queue()
    # Shared latest desired position — RX thread reads this to tag each reply
    shared   = {'des_pos': 0.0}
    interval = 1.0 / UPDATE_HZ

    # ── TX thread ────────────────────────────────────────────────────────────
    def tx_loop():
        next_send = time.perf_counter()
        while not stop.is_set():
            now = time.perf_counter()
            gap = next_send - now
            if gap > 5e-4:
                time.sleep(gap - 4e-4)
            while time.perf_counter() < next_send:
                pass

            t = time.perf_counter() - t0
            des_pos = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t) + CENTER
            des_vel = AMPLITUDE * 2 * math.pi * FREQUENCY * math.cos(2 * math.pi * FREQUENCY * t)
            shared['des_pos'] = des_pos
            try:
                bus.send(can.Message(
                    arbitration_id=CAN_ID,
                    data=pack_cmd(des_pos, des_vel, KP, KD, 0.0),
                    is_extended_id=True,
                ), timeout=0.01)
            except Exception:
                pass
            next_send += interval

    # ── RX thread ────────────────────────────────────────────────────────────
    def rx_loop():
        while not stop.is_set():
            try:
                msg = bus.recv(timeout=0.1)
            except Exception:
                break
            if msg is not None:
                result = decode_reply(msg)
                if result:
                    rx_queue.put((time.perf_counter() - t0, shared['des_pos'], result))

    send_mode(bus, MIT_MODE)
    time.sleep(0.1)

    plot = LivePlot()
    t0   = time.perf_counter()

    tx_thread = threading.Thread(target=tx_loop, daemon=True)
    rx_thread = threading.Thread(target=rx_loop, daemon=True)
    tx_thread.start()
    rx_thread.start()

    print(f"Running sine: amplitude={AMPLITUDE} rad  freq={FREQUENCY} Hz  "
          f"rate={UPDATE_HZ} Hz  — Ctrl+C or Stop button to stop")

    t_print = t0
    try:
        while plot.running:
            # Drain at most 20 frames per outer loop tick so plt.pause() is
            # always reached — an unbounded drain starves the GUI event loop
            # and the Stop button callback never fires.
            for _ in range(20):
                try:
                    t, des_pos, (pos, vel, torque, vbus, temp) = rx_queue.get_nowait()
                except queue.Empty:
                    break
                plot.update(t, des_pos, pos, vel, torque)
                if time.perf_counter() - t_print >= 0.2:
                    print(f"t={t:6.2f}s  des={des_pos:+.3f}  pos={pos:+.3f}  "
                          f"vel={vel:+.3f}  τ={torque:+.3f}  "
                          f"vbus={vbus:.1f}V  temp={temp:.1f}C")
                    t_print = time.perf_counter()
            plt.pause(0.02)  # 20 ms — enough for the Qt/Tk event loop to fire callbacks

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop.set()
        tx_thread.join(timeout=0.5)
        rx_thread.join(timeout=0.5)
        try:
            send_mode(bus, MENU_MODE)
        except Exception:
            pass
        bus.shutdown()
        plot.close()


if __name__ == "__main__":
    main()
