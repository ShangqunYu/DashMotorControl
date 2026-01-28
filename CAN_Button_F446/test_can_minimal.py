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
                
                try:
                    data_bytes = bytes.fromhex(can_data)
                    
                    # Check for startup message
                    if data_bytes[0:3] == b'RDY':
                        print("BOARD READY - waiting for button presses...")
                        print()
                    
                    # Check for button press message
                    elif data_bytes[0:3] == b'BTN':
                        # Extract counter
                        counter = (data_bytes[3] << 24) | (data_bytes[4] << 16) | \
                                 (data_bytes[5] << 8) | data_bytes[6]
                        
                        timestamp = time.strftime('%H:%M:%S')
                        print(f"BUTTON PRESSED! | Press #{counter} | Time: {timestamp}")
                    
                    # Show any other messages
                    else:
                        ascii_text = ''.join([chr(b) if 32 <= b < 127 else '.' 
                                             for b in data_bytes])
                        print(f"ID: 0x{can_id} | Data: {can_data} | ASCII: {ascii_text}")
                        
                except Exception as e:
                    print(f"Parse error: {e}")
                
except KeyboardInterrupt:
    print("Closed")

ser.close()