"""
Quick CAN connectivity test.

Sends a single mode-switch frame to CAN_ID (default 1) on can0.
Put a breakpoint at HAL_CAN_RxFifo0MsgPendingCallback in the firmware —
if the frame arrives the debugger should halt there.

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
CAN_ID  = int(sys.argv[2]) if len(sys.argv) > 2 else 1

# Mode bytes (must match motor_mode_t in foc.h)
TORQUE_CONTROL_MODE  = 0
SPEED_CONTROL_MODE   = 1
POSITION_CONTROL_MODE = 2
CALIBRATION_MODE     = 3
MIT_MODE             = 4
POWER_UP_MODE        = 5
ENCODER_MODE         = 6

def send_mode(bus, can_id, mode):
    msg = can.Message(
        arbitration_id=can_id,
        data=[mode],
        is_extended_id=True,
    )
    bus.send(msg, timeout=0.2)
    print(f"Sent mode-switch frame: id=0x{can_id:03X}  data=[{mode}]  ({msg})")

def main():
    print(f"Opening {CHANNEL} ...")
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as exc:
        sys.exit(f"Failed to open {CHANNEL}: {exc}")

    try:
        send_mode(bus, CAN_ID, MIT_MODE)
    except Exception as exc:
        sys.exit(f"Send failed: {exc}")
    finally:
        bus.shutdown()

    print("Done. Check your debugger breakpoint.")

if __name__ == "__main__":
    main()
