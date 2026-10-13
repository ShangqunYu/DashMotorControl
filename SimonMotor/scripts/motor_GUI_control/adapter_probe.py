#!/usr/bin/env python3
"""Bring-up probe for the AT serial-CAN adapter.

Sends the same logical frame (a param read to node 1) in several candidate
adapter encodings, one at a time, so you can identify which one actually
reaches the motor. Run it with:

  * the firmware built with CAN_DEBUG_ACCEPT_ALL = 1 (can.c) and flashed,
  * a breakpoint on HAL_CAN_RxFifo0MsgPendingCallback (or just watch the
    "rx:" line — variant A elicits a param reply if everything works),
  * the GUI CLOSED (it owns the serial port otherwise).

Interpretation:
  A works                -> everything is fine end to end.
  B works but A doesn't  -> adapter requires a fixed 8-byte data field
                            (real DLC unsupported).
  C works but A doesn't  -> adapter encodes standard IDs as id<<3, not id<<21.
  only D works           -> adapter is configured/hardwired for extended
                            frames only.
  nothing works          -> check CAN1->ESR in the debugger: REC/TEC/LEC all
                            zero means the adapter never transmitted at all
                            (adapter config? termination? baud?).

Each variant carries a distinct first payload byte (0xA1/0xB2/0xC3/0xD4) so
the received frame is identifiable in message_received.data in the debugger.
"""

import sys
import time

from serial import Serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 921600

STD_ID = 0x41         # TYPE_PARAM_RW (4) | node 1  -> arbitration ID 0x041
EXT_ID = 0x1200FD01   # old-style extended ID, recognizably bogus to new firmware


def at_frame(id_field, data):
    data = bytes(data)
    return b"AT" + id_field.to_bytes(4, "big") + bytes([len(data)]) + data + b"\r\n"


VARIANTS = [
    ("A  standard id<<21, DLC 1   (the new protocol as implemented)",
     at_frame(STD_ID << 21, [0xA1])),
    ("B  standard id<<21, DLC 8   (tests: adapter needs fixed 8-byte data)",
     at_frame(STD_ID << 21, [0xB2] + [0] * 7)),
    ("C  standard id<<3,  DLC 8   (tests: alternative standard-ID encoding)",
     at_frame(STD_ID << 3, [0xC3] + [0] * 7)),
    ("D  extended id<<3|4, DLC 8  (the old GUI's frame shape)",
     at_frame((EXT_ID << 3) | 0x4, [0xD4] + [0] * 7)),
    # Kept LAST on purpose: if the adapter mishandles DLC 0 it may wedge its
    # parser and eat later frames — don't let it poison the other variants.
    ("E  standard id<<21, DLC 0   (e-stop shape; suspected adapter trouble)",
     at_frame(0x01 << 21, [])),          # TYPE_ESTOP | node 1, no payload
    ("F  standard id<<21, DLC 1   (repeat of A: did E wedge the adapter?)",
     at_frame(STD_ID << 21, [0xF6])),
]


def main():
    ser = Serial(PORT, BAUD, timeout=0.2)
    print(f"Opened {PORT} @ {BAUD}")
    try:
        for label, frame in VARIANTS:
            print(f"\n-> {label}")
            print(f"   tx: {frame.hex(' ')}")
            ser.reset_input_buffer()
            ser.write(frame)
            time.sleep(0.5)
            rx = ser.read(ser.in_waiting or 1)
            print(f"   rx: {rx.hex(' ') if rx else '(nothing)'}")
            input("   [check breakpoint / CAN1->ESR, then Enter for next] ")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
