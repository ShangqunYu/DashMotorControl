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
import signal
import ctypes
import ctypes.util
import threading
import queue
import collections
import numpy as np
import can
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# GIL-free nanosleep via libc — ctypes releases the GIL before the C call,
# so the TX thread can sleep without blocking other Python threads.
_libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

def _nanosleep(secs):
    if secs > 0:
        ts = _Timespec(int(secs), int((secs % 1) * 1_000_000_000))
        _libc.nanosleep(ctypes.byref(ts), None)  # GIL released here

CHANNEL = sys.argv[1] if len(sys.argv) > 1 else "can0"
CAN_ID  = int(sys.argv[2]) if len(sys.argv) > 2 else 1

# ── Mode bytes (must match foc.h) ────────────────────────────────────────────
MENU_MODE = 0
MIT_MODE  = 5

# ── Scaling ranges — must match flash-stored values on the motor ─────────────
# P_MIN   = -12.5     # rad
# P_MAX   =  12.5     # rad
P_MAX = 12.57
P_MIN = -P_MAX
V_MIN   = -65.0     # rad/s
V_MAX   =  65.0     # rad/s
KP_MAX  =  500.0    # N-m/rad
KD_MAX  =  5.0      # N-m·s/rad
GR = 18.0
KT_AFTER_REDUCER = 2.97
KT = 2.97/GR
I_MAX = 40.0
T_MIN   = -40.0 * GR * KT   # N-m
T_MAX   =  40.0 * GR * KT   # N-m

# ── Sine trajectory parameters ────────────────────────────────────────────────
AMPLITUDE   = 3.14/4.0 # rad   (peak displacement from zero)
FREQUENCY   = 0.2 # Hz
UPDATE_HZ   = 200   # command rate
CENTER      = 0.0   # rad   (center of sine wave)

# ── Control gains ─────────────────────────────────────────────────────────────
KP = 40   # N-m/rad
KD = 1.0    # N-m·s/rad

# ── Plot settings ─────────────────────────────────────────────────────────────
PLOT_WINDOW_S = 5   # seconds of history shown
PLOT_FPS      = 30  # max render rate — independent of UPDATE_HZ


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
        self._t       = collections.deque(maxlen=maxlen)
        self._des     = collections.deque(maxlen=maxlen)
        self._act     = collections.deque(maxlen=maxlen)
        self._des_vel = collections.deque(maxlen=maxlen)
        self._vel     = collections.deque(maxlen=maxlen)
        self._tor     = collections.deque(maxlen=maxlen)
        self._dt      = collections.deque(maxlen=maxlen)
        self._last_render = 0.0
        self.running      = True

        self._app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        pg.setConfigOptions(antialias=True)

        # Outer widget: plots on top, stop button on bottom
        self._win = QtWidgets.QWidget()
        self._win.setWindowTitle("MIT Sine Test")
        self._win.resize(900, 750)
        vbox = QtWidgets.QVBoxLayout(self._win)

        self._gw = pg.GraphicsLayoutWidget()
        vbox.addWidget(self._gw, stretch=1)

        self._btn = QtWidgets.QPushButton("Stop Motor")
        self._btn.clicked.connect(lambda: setattr(self, "running", False))
        vbox.addWidget(self._btn)

        self._win.closeEvent = lambda e: (setattr(self, "running", False), e.accept())
        self._win.show()

        # ── Position subplot ───────────────────────────────────────────────────
        self._ax = self._gw.addPlot(row=0, col=0)
        self._ax.setLabel("left", "position (rotations)")
        self._ax.showGrid(x=True, y=True, alpha=0.3)
        self._ax.addLegend()
        self._line_des = self._ax.plot(pen=pg.mkPen("b", width=2), name="desired")
        self._line_act = self._ax.plot(pen=pg.mkPen((255, 165, 0), width=2), name="actual")

        # ── Velocity subplot ──────────────────────────────────────────────────
        self._ax_vel = self._gw.addPlot(row=1, col=0)
        self._ax_vel.setLabel("left", "velocity (rotations/s)")
        self._ax_vel.showGrid(x=True, y=True, alpha=0.3)
        self._ax_vel.addLegend()
        self._line_des_vel = self._ax_vel.plot(pen=pg.mkPen("b", width=2), name="desired")
        self._line_vel     = self._ax_vel.plot(pen=pg.mkPen((148, 0, 211), width=2), name="actual")

        # ── Torque subplot ────────────────────────────────────────────────────
        self._ax_tor = self._gw.addPlot(row=2, col=0)
        self._ax_tor.setLabel("left", "torque (N·m)")
        self._ax_tor.showGrid(x=True, y=True, alpha=0.3)
        self._line_tor = self._ax_tor.plot(pen=pg.mkPen("r", width=2))

        # ── dt subplot ────────────────────────────────────────────────────────
        self._ax_dt = self._gw.addPlot(row=3, col=0)
        self._ax_dt.setLabel("left", "dt (s)")
        self._ax_dt.setLabel("bottom", "time (s)")
        self._ax_dt.showGrid(x=True, y=True, alpha=0.3)
        self._ax_dt.addLegend()
        self._line_dt = self._ax_dt.plot(pen=pg.mkPen("g", width=2), name="dt")
        self._target_line = pg.InfiniteLine(
            pos=1.0 / UPDATE_HZ,
            angle=0,
            pen=pg.mkPen("gray", width=1, style=QtCore.Qt.DashLine),
            label=f"target ({1000/UPDATE_HZ:.1f} ms)",
            labelOpts={"color": (150, 150, 150), "position": 0.9},
        )
        self._ax_dt.addItem(self._target_line)

        # Link x-axes so all subplots scroll together; disable x auto-range so
        # the window slides instead of growing (we set it explicitly each render).
        for ax in (self._ax_vel, self._ax_tor, self._ax_dt):
            ax.setXLink(self._ax)
        self._ax.enableAutoRange(axis="x", enable=False)

    def update(self, t, des, des_vel, pos, vel, torque):
        dt = t - self._t[-1] if self._t else 0.0
        self._t.append(t)
        self._des.append(des)
        self._act.append(pos)
        self._des_vel.append(des_vel)
        self._vel.append(vel)
        self._tor.append(torque)
        self._dt.append(dt)
        now = time.perf_counter()
        if now - self._last_render >= 1.0 / PLOT_FPS:
            self._last_render = now
            t_arr = np.array(self._t)
            self._line_des.setData(t_arr, np.array(self._des))
            self._line_act.setData(t_arr, np.array(self._act))
            self._line_des_vel.setData(t_arr, np.array(self._des_vel))
            self._line_vel.setData(t_arr, np.array(self._vel))
            self._line_tor.setData(t_arr, np.array(self._tor))
            self._line_dt.setData(t_arr, np.array(self._dt))
            if len(t_arr):
                self._ax.setXRange(t_arr[-1] - PLOT_WINDOW_S, t_arr[-1], padding=0)

    def process_events(self):
        self._app.processEvents()

    def close(self):
        if self._win.isVisible():
            self._app.exec_()


def main():
    print(f"Opening {CHANNEL} ...")
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as exc:
        sys.exit(f"Failed to open {CHANNEL}: {exc}")

    stop     = threading.Event()
    rx_queue = queue.Queue()
    # Shared latest desired position — RX thread reads this to tag each reply
    shared   = {'des_pos': 0.0, 'des_vel': 0.0}
    interval = 1.0 / UPDATE_HZ

    # ── Diagnostics ──────────────────────────────────────────────────────────
    diag_lock    = threading.Lock()
    tx_times     = collections.deque()          # absolute send timestamps
    rtt_samples  = collections.deque(maxlen=500)  # RTT in ms
    tx_intervals = collections.deque(maxlen=500)  # actual TX interval in ms

    # ── TX thread ────────────────────────────────────────────────────────────
    def tx_loop():
        next_send  = time.perf_counter()
        last_tx    = None
        while not stop.is_set():
            now = time.perf_counter()
            gap = next_send - now
            if gap < -interval:
                # Fell more than one tick behind — resync to avoid a burst.
                next_send = now
            elif gap > 0:
                _nanosleep(gap)  # GIL-free; ~50 µs precision, no busy-wait needed

            t = time.perf_counter() - t0
            des_pos = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t) + CENTER
            des_vel = AMPLITUDE * 2 * math.pi * FREQUENCY * math.cos(2 * math.pi * FREQUENCY * t)
            # overwrite
            # des_pos = 0.0
            # des_vel = 0.0
            # KP = 0.0
            # KD = 0.0
            torque_ff = 0.0
            shared['des_pos'] = des_pos
            shared['des_vel'] = des_vel

            t_send = time.perf_counter()
            try:
                bus.send(can.Message(
                    arbitration_id=CAN_ID,
                    data=pack_cmd(des_pos, des_vel, KP, KD, torque_ff),
                    is_extended_id=True,
                ), timeout=0.01)
                with diag_lock:
                    tx_times.append(t_send)
                    if last_tx is not None:
                        tx_intervals.append((t_send - last_tx) * 1e3)
                last_tx = t_send
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
                t_rx = time.perf_counter()
                result = decode_reply(msg)
                if result:
                    with diag_lock:
                        # Discard TX timestamps older than 3 intervals — the motor
                        # dropped those frames and will never reply to them.
                        cutoff = t_rx - 3.0 / UPDATE_HZ
                        while tx_times and tx_times[0] < cutoff:
                            tx_times.popleft()
                        if tx_times:
                            rtt_samples.append((t_rx - tx_times.popleft()) * 1e3)
                    rx_queue.put((t_rx - t0, shared['des_pos'], shared['des_vel'], result))

    # Shrink the GIL switch interval so the main thread's Qt work can't freeze
    # the TX busy-wait for more than ~0.1 ms (default is 5 ms).
    sys.setswitchinterval(0.0001)

    send_mode(bus, MIT_MODE)
    time.sleep(0.1)

    plot = LivePlot()
    t0   = time.perf_counter()

    tx_thread = threading.Thread(target=tx_loop, daemon=True)
    rx_thread = threading.Thread(target=rx_loop, daemon=True)
    tx_thread.start()
    rx_thread.start()

    # print(f"Running sine: amplitude={AMPLITUDE} rad  freq={FREQUENCY} Hz  "
    #       f"rate={UPDATE_HZ} Hz  — Ctrl+C or Stop button to stop")

    t_print  = t0
    t_stats  = t0
    MAX_BACKLOG = max(2, UPDATE_HZ // 20)  # ~50 ms worth of frames

    # ── Drain + render callback (runs every 10 ms via QTimer) ────────────────
    # Between callbacks Qt runs its native C++ event loop and holds no Python
    # GIL, so the TX busy-wait thread is never frozen by GUI work.
    def on_tick():
        nonlocal t_print, t_stats
        if not plot.running:
            drain_timer.stop()
            plot._app.quit()   # exit app.exec_() → finally block runs
            return

        q_depth = rx_queue.qsize()
        while q_depth > MAX_BACKLOG:
            try:
                rx_queue.get_nowait()
                q_depth -= 1
            except queue.Empty:
                break

        while True:
            try:
                t, des_pos, des_vel, (pos, vel, torque, vbus, temp) = rx_queue.get_nowait()
                des_pos = des_pos / (2 * math.pi)
                des_vel = des_vel / (2 * math.pi)
                pos     = pos     / (2 * math.pi)
                vel     = vel     / (2 * math.pi)
            except queue.Empty:
                break
            plot.update(t, des_pos, des_vel, pos, vel, torque)
            if time.perf_counter() - t_print >= 0.2:
                # print(f"t={t:6.2f}s  des={des_pos:+.3f}  pos={pos:+.3f}  "
                #       f"vel={vel:+.3f}  τ={torque:+.3f}  "
                #       f"vbus={vbus:.1f}V  temp={temp:.1f}C")
                t_print = time.perf_counter()

        now = time.perf_counter()
        if now - t_stats >= 1.0:
            t_stats = now
            with diag_lock:
                rtts = list(rtt_samples)
                txis = list(tx_intervals)
            if rtts:
                print(f"  [diag] RTT  min={min(rtts):.2f}  mean={sum(rtts)/len(rtts):.2f}"
                      f"  max={max(rtts):.2f} ms  (n={len(rtts)})")
            if txis:
                print(f"  [diag] TX Δt min={min(txis):.2f}  mean={sum(txis)/len(txis):.2f}"
                      f"  max={max(txis):.2f} ms  target={1000/UPDATE_HZ:.2f} ms")
            print(f"  [diag] queue depth at drain = {q_depth} frames"
                  f"  ({q_depth * 1000 / UPDATE_HZ:.1f} ms backlog)")

    drain_timer = QtCore.QTimer()
    drain_timer.timeout.connect(on_tick)
    drain_timer.start(10)   # 10 ms pump; rendering is time-gated inside update()

    # Allow Ctrl+C to stop the motor without losing the Qt event loop
    signal.signal(signal.SIGINT, lambda *_: setattr(plot, 'running', False))
    _sigint_wakeup = QtCore.QTimer()
    _sigint_wakeup.timeout.connect(lambda: None)  # wake Python every 100 ms to check signals
    _sigint_wakeup.start(100)

    try:
        plot._app.exec_()
    finally:
        drain_timer.stop()
        stop.set()
        tx_thread.join(timeout=0.5)
        rx_thread.join(timeout=0.5)
        try:
            send_mode(bus, MENU_MODE)
        except Exception:
            pass
        bus.shutdown()
        plot.close()   # re-enters exec_() so the user can inspect the final plot


if __name__ == "__main__":
    main()
