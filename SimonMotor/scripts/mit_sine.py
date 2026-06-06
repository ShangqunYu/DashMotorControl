"""
MIT-mode sine trajectory test.

Switches the motor to MIT mode, then streams position commands following a
sine wave.  Press Ctrl+C to stop — the motor is returned to MENU_MODE before
the script exits.

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
AMPLITUDE   = 3.14   # rad   (peak displacement from zero)
FREQUENCY   = 0.1   # Hz
UPDATE_HZ   = 200   # command rate
CENTER      = 0   # rad   (center of sine wave; set to current position to avoid jumps)

# ── Control gains ─────────────────────────────────────────────────────────────
KP = 20   # N-m/rad
KD = 0.2    # N-m·s/rad


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))

def uint_to_float(x, x_min, x_max, bits):
    return x * (x_max - x_min) / ((1 << bits) - 1) + x_min


def pack_cmd(p, v, kp, kd, t_ff):
    """Pack MIT command into 8 bytes matching unpack_cmd() in can.c."""
    p_int  = float_to_uint(p,  P_MIN,  P_MAX,  16)
    v_int  = float_to_uint(v,  V_MIN,  V_MAX,  12)
    kp_int = float_to_uint(kp, 0,      KP_MAX, 12)
    kd_int = float_to_uint(kd, 0,      KD_MAX, 12)
    t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12)
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
    msg = can.Message(
        arbitration_id=CAN_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode],
        is_extended_id=True,
    )
    bus.send(msg, timeout=0.2)


def send_mit(bus, p, v, kp, kd, t_ff=0.0):
    msg = can.Message(
        arbitration_id=CAN_ID,
        data=pack_cmd(p, v, kp, kd, t_ff),
        is_extended_id=True,
    )
    bus.send(msg, timeout=0.2)


def decode_reply(msg):
    d = msg.data
    if len(d) < 8:
        return None
    motor_id = d[0]
    p_int    = (d[1] << 8) | d[2]
    v_int    = (d[3] << 4) | (d[4] >> 4)
    t_int    = ((d[4] & 0xF) << 8) | d[5]
    position = uint_to_float(p_int, P_MIN, P_MAX, 16)
    velocity = uint_to_float(v_int, V_MIN, V_MAX, 12)
    torque   = uint_to_float(t_int, T_MIN, T_MAX, 12)
    vbus     = uint_to_float(d[6], 0.0, 60.0, 8)
    temp     = uint_to_float(d[7], -40.0, 125.0, 8)
    return motor_id, position, velocity, torque, vbus, temp


PLOT_WINDOW_S    = 10    # seconds of history shown
PLOT_UPDATE_EVERY = 5   # update plot every N control iterations (~20 Hz)


class LivePlot:
    def __init__(self, window_s=PLOT_WINDOW_S, update_every=PLOT_UPDATE_EVERY):
        self._every   = update_every
        self._counter = 0
        maxlen = int(window_s * UPDATE_HZ)
        self._t   = collections.deque(maxlen=maxlen)
        self._des = collections.deque(maxlen=maxlen)
        self._act = collections.deque(maxlen=maxlen)
        self._vel = collections.deque(maxlen=maxlen)
        self._tor = collections.deque(maxlen=maxlen)
        self._dt  = collections.deque(maxlen=maxlen)

        self.should_be_running = True

        plt.ion()
        self._fig, (self._ax, self._ax_vel, self._ax_tor, self._ax_dt) = plt.subplots(4, 1, sharex=True)
        self._fig.subplots_adjust(bottom=0.12, hspace=0.45)

        self._line_des, = self._ax.plot([], [], label="desired", color="tab:blue",   marker=".", markersize=6)
        self._line_act, = self._ax.plot([], [], label="actual",  color="tab:orange", marker=".", markersize=6)
        self._ax.set_ylabel("position (rad)")
        self._ax.legend()
        self._ax.grid(True)

        self._line_vel, = self._ax_vel.plot([], [], color="tab:purple", marker=".", markersize=6)
        self._ax_vel.set_ylabel("velocity (rad/s)")
        self._ax_vel.grid(True)

        self._line_tor, = self._ax_tor.plot([], [], color="tab:red", marker=".", markersize=6)
        self._ax_tor.set_ylabel("torque (N·m)")
        self._ax_tor.grid(True)

        self._line_dt, = self._ax_dt.plot([], [], color="tab:green", marker=".", markersize=6)
        self._ax_dt.axhline(1.0 / UPDATE_HZ, color="gray", linestyle="--", linewidth=1, label=f"target ({1000/UPDATE_HZ:.1f} ms)")
        self._ax_dt.set_xlabel("time (s)")
        self._ax_dt.set_ylabel("dt (s)")
        self._ax_dt.legend()
        self._ax_dt.grid(True)

        ax_btn = self._fig.add_axes([0.4, 0.02, 0.2, 0.04])
        self._btn = Button(ax_btn, "Stop Motor")
        self._btn.on_clicked(lambda _: setattr(self, "should_be_running", False))

    def update(self, t, des, actual, vel, torque):
        dt = t - self._t[-1] if self._t else 0.0
        self._t.append(t)
        self._des.append(des)
        self._act.append(actual)
        self._vel.append(vel)
        self._tor.append(torque)
        self._dt.append(dt)
        self._counter += 1
        if self._counter % self._every == 0:
            self._line_des.set_data(self._t, self._des)
            self._line_act.set_data(self._t, self._act)
            self._line_vel.set_data(self._t, self._vel)
            self._line_tor.set_data(self._t, self._tor)
            self._line_dt.set_data(self._t, self._dt)
            for ax in (self._ax, self._ax_vel, self._ax_tor, self._ax_dt):
                ax.relim()
                ax.autoscale_view()
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

    try:
        print("Switching to MIT mode ...")
        send_mode(bus, MIT_MODE)
        time.sleep(0.1)

        plot = LivePlot()
        dt = 1.0 / UPDATE_HZ
        t0 = time.monotonic()
        print(f"Running sine: amplitude={AMPLITUDE} rad  freq={FREQUENCY} Hz  — Ctrl+C to stop")

        while plot.should_be_running:
            t = time.monotonic() - t0
            des_pos = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t) +CENTER
            des_vel = 0  #AMPLITUDE * 2 * math.pi * FREQUENCY * math.cos(2 * math.pi * FREQUENCY * t)

            send_mit(bus, des_pos, des_vel, KP, KD)
            # breakpoint()

            reply = bus.recv(timeout=dt)
            if reply is not None:
                result = decode_reply(reply)
                if result:
                    mid, pos, vel, torque, vbus, temp = result
                    print(f"t={t:6.2f}s  des={des_pos:+.3f}  pos={pos:+.3f}  vel={vel:+.3f}  τ={torque:+.3f}  vbus={vbus:.1f}V  temp={temp:.1f}C")
                    plot.update(t, des_pos, pos, vel, torque)

            elapsed = time.monotonic() - t0 - t
            remaining = dt - elapsed
            if remaining > 0:
                plt.pause(remaining)

        print("\nStopping — sending MENU_MODE ...")
        send_mode(bus, MENU_MODE)
        plot.close()

    except KeyboardInterrupt:
        print("\nStopping — sending MENU_MODE ...")
        send_mode(bus, MENU_MODE)
    except Exception as exc:
        print(f"Error: {exc}")
        send_mode(bus, MENU_MODE)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
