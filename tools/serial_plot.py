#!/usr/bin/env python3
"""
Live plotter for float32 values coming from a serial port.

Usage:
  python tools/serial_plot.py --port COM8 --baud 115200

Reads 4-byte IEEE-754 floats (little-endian) and shows a live-scrolling plot.
Optional: --csv to save received values to a CSV file.

Dependencies:
  pip install pyserial matplotlib

"""

import argparse
import serial
import struct
import sys
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def parse_args():
    p = argparse.ArgumentParser(description="Live plot float32 values from serial port")
    p.add_argument("--port", "-p", default="COM8", help="Serial port (COMx or /dev/ttyUSBx)")
    p.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    p.add_argument("--buf", type=int, default=500, help="Number of samples to keep in buffer")
    p.add_argument("--interval", type=int, default=50, help="Update interval in ms")
    p.add_argument("--csv", default=None, help="Optional CSV output filename")
    return p.parse_args()


class SerialPlot:
    def __init__(self, port, baud, buf_len=500, csv_file=None):
        self.port = port
        self.baud = baud
        self.buf_len = buf_len
        self.csv_file = csv_file

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            print(f"Failed to open serial port {self.port}: {e}")
            raise

        self.buffer = deque([0.0] * buf_len, maxlen=buf_len)
        self.rawbuf = bytearray()
        self.start_time = time.time()

        if self.csv_file:
            self.csv_fh = open(self.csv_file, "wb")
        else:
            self.csv_fh = None

    def close(self):
        if self.csv_fh:
            self.csv_fh.close()
        try:
            self.ser.close()
        except Exception:
            pass

    def read_available(self):
        # Read any available bytes and parse float32s
        try:
            n = self.ser.in_waiting
        except Exception:
            n = 0
        if n:
            data = self.ser.read(n)
            self.rawbuf.extend(data)

        # Extract 4-byte floats (little-endian)
        while len(self.rawbuf) >= 4:
            chunk = bytes(self.rawbuf[:4])
            try:
                val = struct.unpack('<f', chunk)[0]
            except struct.error:
                # badly aligned; discard one byte and continue
                self.rawbuf.pop(0)
                continue
            # consume 4 bytes
            del self.rawbuf[:4]
            self.buffer.append(val)
            if self.csv_fh:
                # write textual CSV line: timestamp,value\n
                ts = time.time() - self.start_time
                line = f"{ts:.6f},{val:.9g}\n".encode('ascii')
                self.csv_fh.write(line)
        
    def get_data(self):
        return list(self.buffer)


def main():
    args = parse_args()

    sp = None
    try:
        sp = SerialPlot(args.port, args.baud, buf_len=args.buf, csv_file=args.csv)
    except Exception:
        sys.exit(1)

    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=1)
    ax.set_xlabel('Samples (most recent on right)')
    ax.set_ylabel('Value')
    ax.grid(True)

    def init():
        x = list(range(-args.buf + 1, 1))
        line.set_data(x, [0] * args.buf)
        ax.set_xlim(-args.buf + 1, 0)
        ax.set_ylim(-2, 2)
        return (line,)

    def update(frame):
        sp.read_available()
        data = sp.get_data()
        x = list(range(-len(data) + 1, 1))
        line.set_data(x, data)

        # autoscale Y with margin (only if we have non-zero data)
        if len(data) > 0 and any(v != 0 for v in data):
            ymin = min(data)
            ymax = max(data)
            if abs(ymax - ymin) < 1e-6:
                # values are essentially constant
                center = (ymin + ymax) / 2
                ymin = center - 0.5
                ymax = center + 0.5
            else:
                vrange = ymax - ymin
                margin = vrange * 0.1
                ymin -= margin
                ymax += margin
            ax.set_ylim(ymin, ymax)
            # Update title to show current range
            ax.set_title(f'Range: [{ymin:.3f}, {ymax:.3f}]')
        return (line,)

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=args.interval, blit=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        sp.close()


if __name__ == '__main__':
    main()
