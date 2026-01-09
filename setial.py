import serial
import struct

ser = serial.Serial('COM8', 115200, timeout=1)

while True:
    # line = ser.readline().decode(errors='ignore').strip()
    data = ser.read(4)        # read 4 bytes
    if len(data) != 4:
        continue  # skip incomplete reads
    val = struct.unpack('f', data)[0]
    print("STM32:", val)


        