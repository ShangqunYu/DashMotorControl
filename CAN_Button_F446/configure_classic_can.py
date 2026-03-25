import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

commands = [
    'can off',
    'conf set can.bitrate 1000000',
    'conf set can.fdcan_frame 0',
    'conf set can.bitrate_switch 0',
    'conf set can.termination 1',
    'conf write',
    'can on'
]

for cmd in commands:
    print(f"> {cmd}")
    ser.write((cmd + '\n').encode())
    time.sleep(0.2)
    response = ser.read_all().decode().strip()
    if response:
        print(f"< {response}")

print("\nFDCANUSB configured for classic CAN 1Mbps")
ser.close()