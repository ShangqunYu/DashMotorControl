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
import can

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
AMPLITUDE   = 2.0   # rad   (peak displacement from zero)
FREQUENCY   = 1   # Hz
UPDATE_HZ   = 100   # command rate

# ── Control gains ─────────────────────────────────────────────────────────────
KP = 20.0   # N-m/rad
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

        dt = 1.0 / UPDATE_HZ
        t0 = time.monotonic()
        print(f"Running sine: amplitude={AMPLITUDE} rad  freq={FREQUENCY} Hz  — Ctrl+C to stop")

        while True:
            t = time.monotonic() - t0
            des_pos = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t)
            des_vel = AMPLITUDE * 2 * math.pi * FREQUENCY * math.cos(2 * math.pi * FREQUENCY * t)

            send_mit(bus, des_pos, des_vel, KP, KD)

            reply = bus.recv(timeout=dt)
            if reply is not None:
                result = decode_reply(reply)
                if result:
                    mid, pos, vel, torque, vbus, temp = result
                    print(f"t={t:6.2f}s  des={des_pos:+.3f}  pos={pos:+.3f}  vel={vel:+.3f}  τ={torque:+.3f}  vbus={vbus:.1f}V  temp={temp:.1f}C")

            elapsed = time.monotonic() - t0 - t
            if elapsed < dt:
                time.sleep(dt - elapsed)

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
