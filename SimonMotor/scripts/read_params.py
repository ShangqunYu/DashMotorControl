"""
Read, write, and reset user config parameters over CAN.

Motor must be in MENU_MODE for config commands to be accepted.
The script switches to MENU_MODE automatically before any operation.

Protocol (matches can.c):
  Read:          [FF x6][FE][param_index]                → reply: [id][idx][float BE][0][0]
  Write:         [FF FF FD][param_index][float BE 4 bytes] → reply: echo of written value
  Factory reset: [FF FF FD][FF][00 00 00 00]              → reply: value at idx 0xFF (0.0)

Param index encoding (matches param_id_t in user_config.h):
  bits[6:0] = array index
  bit[7]    = 0 → __float_reg,  1 → __int_reg

Usage:
    python3 read_params.py [channel] [can_id]            # read all params
    python3 read_params.py [channel] [can_id] --write PARAM_NAME VALUE
    python3 read_params.py [channel] [can_id] --reset

Examples:
    python3 read_params.py
    python3 read_params.py can0 1
    python3 read_params.py can0 1 --write GR 18.0
    python3 read_params.py can0 1 --reset
"""

import sys
import struct
import time
import can

# ── argument parsing ──────────────────────────────────────────────────────────
args = sys.argv[1:]

CHANNEL = "can0"
CFG_CAN_ID  = 1
MODE    = "read"      # "read" | "write" | "reset"
WRITE_NAME  = None
WRITE_VALUE = None

i = 0
while i < len(args):
    if args[i] == "--write" and i + 2 < len(args):
        MODE        = "write"
        WRITE_NAME  = args[i + 1].upper()
        WRITE_VALUE = float(args[i + 2])
        i += 3
    elif args[i] == "--reset":
        MODE = "reset"
        i += 1
    elif not args[i].startswith("--"):
        if   i == 0: CHANNEL = args[i]
        elif i == 1: CFG_CAN_ID  = int(args[i])
        i += 1
    else:
        i += 1

# ── param table ───────────────────────────────────────────────────────────────
# (index, name, unit)  — index mirrors param_id_t in user_config.h
PARAMS = [
    (0,         "CFG_PPAIRS",             "pole pairs"),
    (1,         "CFG_GR",                  "gear ratio"),
    (2,         "CFG_KT",                 "N·m/A  (rotor, before GR)"),
    (3,         "CFG_E_ZERO_RAD",         "rad"),
    (4,         "CFG_M_ZERO_RAD",         "rad"),
    (5,         "CFG_I_MAX",              "A"),
    (6,         "CFG_P_MIN",              "rad"),
    (7,         "CFG_P_MAX",              "rad"),
    (8,         "CFG_V_MIN",              "rad/s"),
    (9,         "CFG_V_MAX",              "rad/s"),
    (10,        "CFG_KP_MAX",             "kP"),
    (11,        "CFG_KD_MAX",             "kD"),
    (12,        "CFG_I_CAL",              "A"),
    (13,        "CFG_KP_DQ",              "kp on d/q axes"),
    (14,        "CFG_KI_DQ",              "ki on d/q axes"),
    (15,        "CFG_TEMP_MIN",             "°C"),
    (16,        "CFG_TEMP_MAX",             "°C"),
    (17,        "CFG_RESISTANCE", "Ω"),
    (18,        "CFG_INDUCTANCE", "H"),
    (0x80 | 0,  "CFG_ENC_SEL",            "0=internal,1=external"),
    (0x80 | 1,  "CFG_PHASE_ORDER",        "0 same order as encoder, 1 reversed"),
    (0x80 | 2,  "CFG_CAN_ID",             ""),
    (0x80 | 3,  "CFG_CAN_MASTER",             ""),
    (0x80 | 4,  "CFG_CAN_TIMEOUT",        "ms"),
    (0x80 | 5,  "CALIBRATION_DONE",   ""),
]

PARAM_BY_NAME = {name: idx for idx, name, _ in PARAMS}

MENU_MODE = 0
TIMEOUT   = 0.15


# ── CAN helpers ───────────────────────────────────────────────────────────────
def send_mode(bus, mode):
    bus.send(can.Message(
        arbitration_id=CFG_CAN_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode],
        is_extended_id=True,
    ), timeout=0.2)


def send_param_read(bus, param_index):
    bus.send(can.Message(
        arbitration_id=CFG_CAN_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, param_index],
        is_extended_id=True,
    ), timeout=0.2)


def send_param_write(bus, param_index, value):
    data = [0xFF, 0xFF, 0xFD, param_index] + list(struct.pack('>f', value))
    bus.send(can.Message(arbitration_id=CFG_CAN_ID, data=data, is_extended_id=True), timeout=0.2)


def send_factory_reset(bus):
    bus.send(can.Message(
        arbitration_id=CFG_CAN_ID,
        data=[0xFF, 0xFF, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=True,
    ), timeout=0.2)


def recv_param_reply(bus, expected_index):
    """Return float value from reply frame, or None on timeout."""
    deadline = time.perf_counter() + TIMEOUT
    while time.perf_counter() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.perf_counter()))
        if msg is None:
            break
        if len(msg.data) >= 6 and msg.data[0] == CFG_CAN_ID and msg.data[1] == expected_index:
            return struct.unpack('>f', bytes(msg.data[2:6]))[0]
    return None


def fmt_value(index, value):
    return f"{int(value)}" if index & 0x80 else f"{value:.4f}"


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    print(f"Opening {CHANNEL}, motor id={CFG_CAN_ID} ...")
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as exc:
        sys.exit(f"Failed to open {CHANNEL}: {exc}")

    try:
        send_mode(bus, MENU_MODE)
        time.sleep(0.05)

        if MODE == "read":
            name_width = max(len(name) for _, name, _ in PARAMS)
            print()
            print(f"  {'Parameter':<{name_width}}   {'Value':>12}  Unit")
            print(f"  {'-'*name_width}   {'-'*12}  ----")
            for index, name, unit in PARAMS:
                send_param_read(bus, index)
                value = recv_param_reply(bus, index)
                if value is None:
                    print(f"  {name:<{name_width}}   {'(no reply)':>12}")
                else:
                    print(f"  {name:<{name_width}}   {fmt_value(index, value):>12}  {unit}")
            print()

        elif MODE == "write":
            if WRITE_NAME not in PARAM_BY_NAME:
                print(f"Unknown param '{WRITE_NAME}'. Valid names:")
                for _, name, _ in PARAMS:
                    print(f"  {name}")
                return
            idx = PARAM_BY_NAME[WRITE_NAME]
            send_param_write(bus, idx, WRITE_VALUE)
            reply = recv_param_reply(bus, idx)
            if reply is None:
                print(f"  {WRITE_NAME}: no reply (motor in MENU_MODE?)")
            else:
                print(f"  {WRITE_NAME} = {fmt_value(idx, reply)}")

        elif MODE == "reset":
            print("Sending factory reset ...")
            send_factory_reset(bus)
            reply = recv_param_reply(bus, 0xFF)
            if reply is not None:
                print("  Done (motor confirmed reset).")
            else:
                print("  Sent (no confirmation reply).")

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
