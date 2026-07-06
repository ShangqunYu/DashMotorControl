"""
CAN round-trip benchmark — threaded TX / RX.

TX and RX run in separate threads so the send rate is never limited by
waiting for a reply.  RTT is computed by matching TX and RX timestamps in
order (valid because CAN is a single bus and the motor replies in order).

Usage:
    python3 can_bench.py [channel] [can_id] [duration_s] [target_hz] [sine_amp]

Arguments:
    sine_amp  Peak position amplitude in rad for a 1.6 Hz sine command.
              0 (default) sends a fixed hold-zero command.

Examples:
    python3 can_bench.py                       # can0  id=1  5 s  500 Hz  fixed
    python3 can_bench.py can0 1 5 500          # fixed command at 500 Hz
    python3 can_bench.py can0 1 5 500 0.39     # sine wave at 500 Hz
"""

import sys
import math
import time
import statistics
import threading
import collections
import can

CHANNEL   = sys.argv[1]         if len(sys.argv) > 1 else "can0"
CFG_CAN_ID    = int(sys.argv[2])    if len(sys.argv) > 2 else 1
DURATION  = float(sys.argv[3])  if len(sys.argv) > 3 else 500.0
TARGET_HZ = float(sys.argv[4])  if len(sys.argv) > 4 else 500.0   # 0 = unlimited
SINE_AMP  = float(sys.argv[5])  if len(sys.argv) > 5 else 0.6     # 0 = fixed command
SINE_HZ   = 0.8

MENU_MODE = 0
MIT_MODE  = 2

CFG_P_MIN, CFG_P_MAX = -12.57,  12.57
CFG_V_MIN, CFG_V_MAX = -65.0,  65.0
CFG_KP_MAX       =  500.0
CFG_KD_MAX       =  5.0
T_MIN, T_MAX = -18.0,  18.0
kp = 20.0
kd = 0.5


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def pack_cmd(p, v, kp, kd, t_ff):
    p_int  = float_to_uint(p,    CFG_P_MIN, CFG_P_MAX,  16)
    v_int  = float_to_uint(v,    CFG_V_MIN, CFG_V_MAX,  12)
    kp_int = float_to_uint(kp,   0,     CFG_KP_MAX, 12)
    kd_int = float_to_uint(kd,   0,     CFG_KD_MAX, 12)
    t_int  = float_to_uint(t_ff, T_MIN, T_MAX,  12)
    return bytes([
        p_int >> 8,
        p_int & 0xFF,
        v_int >> 4,
        ((v_int & 0xF) << 4) | (kp_int >> 8),
        kp_int & 0xFF,
        kd_int >> 4,
        ((kd_int & 0xF) << 4) | (t_int >> 8),
        t_int & 0xFF,
    ])


CMD_BYTES = pack_cmd(0.0, 0.0, 0.0, 0.1, 0.0)   # hold zero, light damping


def send_mode(bus, mode):
    bus.send(can.Message(
        arbitration_id=CFG_CAN_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode],
        is_extended_id=True,
    ), timeout=0.2)


def main():
    print(f"Opening {CHANNEL}, id={CFG_CAN_ID} ...")
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as exc:
        sys.exit(f"Failed to open {CHANNEL}: {exc}")

    interval  = (1.0 / TARGET_HZ) if TARGET_HZ > 0 else 0.0
    stop      = threading.Event()
    tx_times  = collections.deque()   # perf_counter timestamps, one per sent frame
    rx_times  = collections.deque()   # perf_counter timestamps, one per received frame
    lock      = threading.Lock()

    # ── TX thread ────────────────────────────────────────────────────────────
    def tx_loop():
        next_send = time.perf_counter()
        msg = can.Message(arbitration_id=CFG_CAN_ID, data=CMD_BYTES, is_extended_id=True)
        while not stop.is_set():
            now = time.perf_counter()
            if interval > 0:
                gap = next_send - now
                if gap > 5e-4:          # > 0.5 ms left: sleep
                    time.sleep(gap - 4e-4)
                while time.perf_counter() < next_send:  # busy-wait the last ~0.4 ms
                    pass
            if SINE_AMP > 0:
                elapsed = time.perf_counter() - t_start
                pos = SINE_AMP * math.sin(2 * math.pi * SINE_HZ * elapsed)
                vel = SINE_AMP * 2 * math.pi * SINE_HZ * math.cos(2 * math.pi * SINE_HZ * elapsed)
                msg = can.Message(
                    arbitration_id=CFG_CAN_ID,
                    data=pack_cmd(pos, vel, kp, kd, 0.0),
                    is_extended_id=True,
                )
            t = time.perf_counter()  # stamp right before send, not before sine math
            try:
                bus.send(msg, timeout=0.01)
                with lock:
                    tx_times.append(t)
            except Exception:
                pass
            next_send = t + interval

    # ── RX thread ────────────────────────────────────────────────────────────
    def rx_loop():
        while True:
            try:
                msg = bus.recv(timeout=0.1)
            except Exception:
                break                          # socket shut down
            if msg is not None and len(msg.data) >= 6:
                with lock:
                    rx_times.append(time.perf_counter())
            if stop.is_set() and msg is None:
                break                          # stop set + recv timed out → nothing in-flight

    send_mode(bus, MIT_MODE)
    time.sleep(0.05)

    t_start  = time.perf_counter()
    t_status = t_start

    threading.Thread(target=tx_loop, daemon=True).start()
    threading.Thread(target=rx_loop, daemon=True).start()

    rate_str = "unlimited" if TARGET_HZ == 0 else f"{TARGET_HZ:.0f} Hz"
    cmd_str  = f"sine {SINE_AMP:.3f} rad @ {SINE_HZ} Hz" if SINE_AMP > 0 else "fixed (hold zero)"
    print(f"Benchmarking for {DURATION:.0f} s  (target: {rate_str},  cmd: {cmd_str}) ...")
    print()

    t_end = None

    try:
        while time.perf_counter() - t_start < DURATION:
            time.sleep(0.1)
            now = time.perf_counter()
            if now - t_status >= 1.0:
                elapsed = now - t_start
                with lock:
                    n_tx = len(tx_times)
                    n_rx = len(rx_times)
                print(f"  {elapsed:4.1f}s  TX {n_tx/elapsed:6.0f} Hz  "
                      f"RX {n_rx/elapsed:6.0f} Hz")
                t_status = now
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        t_end = time.perf_counter()
        stop.set()
        time.sleep(0.05)   # drain in-flight frames before shutdown (max RTT ~2 ms)
        try:
            send_mode(bus, MENU_MODE)
        except Exception:
            pass
        bus.shutdown()

    elapsed = t_end - t_start

    with lock:
        tx_list = list(tx_times)
        rx_list = list(rx_times)

    n_tx = len(tx_list)
    n_rx = len(rx_list)

    # Match TX→RX in order to compute per-frame RTT.
    # For each TX timestamp find the earliest RX timestamp that follows it.
    rtts = []
    rx_idx = 0
    for t_s in tx_list:
        while rx_idx < len(rx_list) and rx_list[rx_idx] < t_s:
            rx_idx += 1
        if rx_idx < len(rx_list):
            rtts.append((rx_list[rx_idx] - t_s) * 1_000)
            rx_idx += 1

    print()
    print("=" * 54)
    print(f"  Duration       : {elapsed:.2f} s")
    print(f"  TX frames      : {n_tx}  ({n_tx/elapsed:.1f} Hz)")
    print(f"  RX frames      : {n_rx}  ({n_rx/elapsed:.1f} Hz)")
    print(f"  Dropped        : {n_tx - n_rx}  ({100*(n_tx-n_rx)/max(n_tx,1):.1f}%)")
    if rtts:
        rtts_sorted = sorted(rtts)
        p50  = rtts_sorted[int(len(rtts_sorted) * 0.50)]
        p99  = rtts_sorted[int(len(rtts_sorted) * 0.99)]
        p999 = rtts_sorted[min(int(len(rtts_sorted) * 0.999), len(rtts_sorted)-1)]
        print(f"  RTT min        : {min(rtts):.3f} ms")
        print(f"  RTT p50        : {p50:.3f} ms")
        print(f"  RTT mean       : {statistics.mean(rtts):.3f} ms")
        print(f"  RTT p99        : {p99:.3f} ms")
        print(f"  RTT p99.9      : {p999:.3f} ms")
        print(f"  RTT max        : {max(rtts):.3f} ms")
        print(f"  RTT stdev      : {statistics.stdev(rtts):.3f} ms")
        print()
        # Auto-scale histogram step to fit [0, max_rtt] in 7 buckets.
        # Step is rounded up to the nearest 1-2-5 * 10^n for clean labels.
        raw_step = max(rtts) / 7
        exp = 10 ** math.floor(math.log10(raw_step))
        step = next(s * exp for s in (1, 2, 5, 10) if s * exp >= raw_step)
        edges  = [step * i for i in range(1, 8)] + [float("inf")]
        labels = [f"<{step:.4g}"] + \
                 [f"{step*i:.4g}-{step*(i+1):.4g}" for i in range(1, 7)] + \
                 [f">{step*7:.4g}"]
        counts = [0] * 8
        for r in rtts:
            for i, edge in enumerate(edges):
                if r < edge:
                    counts[i] += 1
                    break
        lw = max(len(l) for l in labels)
        print("  RTT histogram (ms):")
        for label, cnt in zip(labels, counts):
            bar = "█" * (cnt * 30 // (max(counts) or 1))
            print(f"    {label:>{lw}}  {bar:<30}  {cnt}")
    print("=" * 54)


if __name__ == "__main__":
    main()
