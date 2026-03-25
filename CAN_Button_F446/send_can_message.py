import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

commands = "can std 01 024900"

for i in range(500):
    print(f"> {commands}")
    ser.write((commands + '\n').encode())
    time.sleep(1.0)
    response = ser.read_all().decode().strip()
    if response:
        print(f"< {response}")


ser.close()