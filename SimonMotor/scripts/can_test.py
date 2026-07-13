"""
Quick CAN connectivity test.

Sends a mode-switch frame to CFG_CAN_ID (default 1) on can0, then listens for
the motor's reply frame and decodes it.

Usage:
    python3 can_test.py [channel] [can_id]

Examples:
    python3 can_test.py              # can0, id=1
    python3 can_test.py can0 1
    python3 can_test.py can1 3
"""

import sys
import can

CHANNEL = sys.argv[1] if len(sys.argv) > 1 else "can0"
CFG_CAN_ID  = int(sys.argv[2]) if len(sys.argv) > 2 else 1

# Mode bytes (must match state in PMSM_motor.h)
MENU_MODE = 0
CALIBRATION_MODE     = 1
MIT_MODE             = 2
ENCODER_MODE         = 3
SET_ZERO_MODE        = 4
R_MEAS_MODE          = 5
L_MEAS_MODE          = 6

# Scaling ranges — must match flash-stored values on the motor
# V_BUS and TEMP are hardcoded in hw_config.h; the rest are user-configurable
CFG_P_MIN   = -12.57     # rad
CFG_P_MAX   =  12.57     # rad
CFG_V_MIN   = -45.0     # rad/s
CFG_V_MAX   =  45.0     # rad/s
T_MIN   = -18.0     # N-m  (I_MAX * KT * CFG_GR, negated)
T_MAX   =  18.0     # N-m


def uint_to_float(x, x_min, x_max, bits):
    span = x_max - x_min
    return x * span / ((1 << bits) - 1) + x_min


def send_mode(bus, can_id, mode):
    msg = can.Message(
        arbitration_id=can_id,
        data=[255,255,255,255,255,255,255,mode],
        is_extended_id=True,
    )
    bus.send(msg, timeout=0.2)
    print(f"Sent mode-switch frame: id=0x{can_id:03X}  data={list(msg.data)}")


def receive_reply(bus, timeout=1.0):
    msg = bus.recv(timeout=timeout)
    if msg is None:
        print("No reply received within timeout.")
        return

    d = msg.data
    print(f"Received frame: id=0x{msg.arbitration_id:03X}  data={list(d)}")

    if len(d) < 8:
        print("  (frame too short to decode)")
        return

    motor_id = d[0]
    p_int    = (d[1] << 8) | d[2]
    v_int    = (d[3] << 4) | (d[4] >> 4)
    t_int    = ((d[4] & 0xF) << 8) | d[5]
    vb_int   = d[6]
    temp_int = d[7]

    position = uint_to_float(p_int, CFG_P_MIN, CFG_P_MAX, 16)
    velocity = uint_to_float(v_int, CFG_V_MIN, CFG_V_MAX, 12)
    torque   = uint_to_float(t_int, T_MIN, T_MAX, 12)
    vbus     = uint_to_float(vb_int, 0.0, 60.0, 8)
    temp     = uint_to_float(temp_int, -40.0, 125.0, 8)

    print(f"  motor_id = {motor_id}")
    print(f"  position = {position:.4f} rad")
    print(f"  velocity = {velocity:.4f} rad/s")
    print(f"  torque   = {torque:.4f} N-m")
    print(f"  vbus     = {vbus:.2f} V")
    print(f"  temp     = {temp:.1f} C")


def main():
    print(f"Opening {CHANNEL} ...")
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as exc:
        sys.exit(f"Failed to open {CHANNEL}: {exc}")

    try:
        send_mode(bus, CFG_CAN_ID, R_MEAS_MODE)
        receive_reply(bus)
    except Exception as exc:
        sys.exit(f"Error: {exc}")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
