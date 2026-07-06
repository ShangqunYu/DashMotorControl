"""
CAN round-trip latency sampler.

Sends one MIT frame, waits for the reply, records the time, repeats N times,
then prints the average (and min/max).

Usage:
    python3 can_rtt.py [channel] [can_id] [samples]

Examples:
    python3 can_rtt.py
    python3 can_rtt.py can0 1 100
"""

import sys
import time
import can

CHANNEL = sys.argv[1]       if len(sys.argv) > 1 else "can0"
CFG_CAN_ID  = int(sys.argv[2])  if len(sys.argv) > 2 else 1
SAMPLES = int(sys.argv[3])  if len(sys.argv) > 3 else 100

MENU_MODE = 0
MIT_MODE  = 5
RECV_TIMEOUT = 0.05   # 50 ms — generous enough to never miss a reply


def send_mode(bus, mode):
    bus.send(can.Message(
        arbitration_id=CFG_CAN_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode],
        is_extended_id=True,
    ), timeout=0.2)


# Zero-position command, light damping so the motor stays calm
CMD = can.Message(
    arbitration_id=CFG_CAN_ID,
    data=[0x7F, 0xFF, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00],
    is_extended_id=True,
)


def main():
    print(f"Opening {CHANNEL}, id={CFG_CAN_ID} ...")
    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as exc:
        sys.exit(f"Failed to open {CHANNEL}: {exc}")

    try:
        send_mode(bus, MIT_MODE)
        time.sleep(0.05)

        rtts = []
        print(f"Sending {SAMPLES} frames ...\n")

        for i in range(SAMPLES):
            t0 = time.perf_counter()
            bus.send(CMD, timeout=0.1)
            reply = bus.recv(timeout=RECV_TIMEOUT)
            t1 = time.perf_counter()

            if reply is None:
                print(f"  [{i+1:3d}]  NO REPLY (timeout)")
                continue

            rtt_ms = (t1 - t0) * 1000
            rtts.append(rtt_ms)
            print(f"  [{i+1:3d}]  {rtt_ms:.3f} ms")

            time.sleep(0.002)   # 2 ms gap between frames

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        send_mode(bus, MENU_MODE)
        bus.shutdown()

    if not rtts:
        print("No replies received.")
        return

    print(f"\n--- {len(rtts)} / {SAMPLES} replies received ---")
    print(f"  avg  {sum(rtts)/len(rtts):.3f} ms")
    print(f"  min  {min(rtts):.3f} ms")
    print(f"  max  {max(rtts):.3f} ms")


if __name__ == "__main__":
    main()
