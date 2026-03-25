import serial
import time

print("WAITING FOR BUTTON PRESSES FROM STM32")
print("\nPress the button on your board...\n")

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

# Ensure CAN is on
ser.write(b'can on\n')
time.sleep(0.2)
ser.read_all()

try:
    while True:
        line = ser.readline().decode().strip()
        
        if line and line.startswith('rcv'):
            parts = line.split()
            if len(parts) >= 3:
                can_id = parts[1]
                can_data = parts[2]
                
                data_bytes = bytes.fromhex(can_data)
                
                print(f"Received CAN message: ID={can_id}, Data={data_bytes.hex()}")
                
except KeyboardInterrupt:
    print("Closed")

ser.close()