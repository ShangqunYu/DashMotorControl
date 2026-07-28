"""Serial transport + CAN/MIT protocol for the motor controller.

`MotorLink` owns the serial port and the background reader thread and exposes a
UI-agnostic API (send command, set mode, read/write config parameters). Decoded
status frames and human-readable status messages are delivered through Qt
signals so the GUI thread never touches the port directly.
"""

import struct
import time
import threading
from contextlib import contextmanager

from serial import Serial
from PyQt5.QtCore import QObject, pyqtSignal

import protocol
from config import REPLY_FIELDS, AUTO_SEND_MS


class MotorLink(QObject):
    # Emitted from the reader thread with a decoded status dict
    # ({field: value, ..., 'time': seconds_since_start}).
    data_received = pyqtSignal(dict)
    # Emitted with a human-readable connection/activity message.
    status_changed = pyqtSignal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate

        # MIT command/decode ranges. None until the parameters are read off the
        # motor; commands are refused and replies aren't decoded until then.
        self.scaling = None

        self.serial = None
        self.running = False
        self._serial_lock = threading.Lock()
        # When set, the reader stops consuming so a config transaction can
        # capture its own reply instead of it being swallowed by the plot path.
        self._config_busy = threading.Event()
        self._read_thread = None
        self._start_time = None

        # Command streaming (runs in its own thread so a busy GUI event loop
        # can't starve the firmware's CAN-timeout watchdog).
        self._stream_enabled = threading.Event()
        self._stream_cmd = None       # (can_id, {p, v, kp, kd, t_ff}) or None
        self._stream_error = None     # last emitted error, to avoid spamming
        self._stream_thread = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def connect(self):
        """Open the serial port. Returns True on success."""
        try:
            self.serial = Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            self.status_changed.emit(f"Connected to {self.port}")
            return True
        except Exception as e:
            self.running = False
            self.status_changed.emit(f"Failed to connect: {e}")
            return False

    def start(self):
        """Start the background reader and streamer threads (no-op if not connected)."""
        if not self.serial:
            return
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

    def close(self):
        self.running = False
        if self.serial:
            self.serial.close()

    # ── Low-level helpers ─────────────────────────────────────────────────────
    @staticmethod
    def _hex_bytes(data_bytes):
        return " ".join(f"{b:02X}" for b in data_bytes)

    @staticmethod
    def _extract_can_payload(raw_data):
        if raw_data is None or len(raw_data) <= 9:
            return None
        return raw_data[7:-2]

    def _clear_serial_buffer(self):
        if not self.serial:
            return
        try:
            with self._serial_lock:
                if self.serial.in_waiting:
                    self.serial.read(self.serial.in_waiting)
        except Exception:
            pass

    def _write(self, serial_cmd):
        """Thread-safe raw write to the port."""
        if not self.serial:
            raise ConnectionError("Serial port is not connected")
        with self._serial_lock:
            self.serial.write(serial_cmd)

    def _send_can_frame(self, can_id, data_bytes, timeout=0.25):
        """Send a CAN frame and wait up to `timeout` for a reply payload."""
        if not self.serial:
            raise ConnectionError("Serial port is not connected")

        serial_cmd = protocol.can2serial(can_id, self._hex_bytes(data_bytes))
        deadline = time.time() + timeout
        buffer = b""

        with self._serial_lock:
            self.serial.write(serial_cmd)
            time.sleep(0.05)

        while time.time() < deadline:
            with self._serial_lock:
                n = self.serial.in_waiting
                if n:
                    buffer += self.serial.read(n)
                    payload = self._extract_can_payload(buffer)
                    if payload and len(payload) >= 8:
                        return payload
            time.sleep(0.1)

        return None

    def _send_mode_frame(self, mode, can_id):
        if not self.serial:
            raise ConnectionError("Serial port is not connected")

        mode_data = f"FF FF FF FF FF FF FF {mode:02X}"
        mode_serial = protocol.can2serial(can_id, mode_data)
        with self._serial_lock:
            self.serial.write(mode_serial)
        # Give the adapter time to turn the frame around, WITHOUT holding the
        # lock — a lock-held sleep here stalls the command streamer and eats
        # into the firmware's CAN-timeout budget.
        time.sleep(0.05)
        self._clear_serial_buffer()

    @contextmanager
    def _config_transaction(self, can_id):
        """Pause the background reader for the duration of a config read/write
        so its reply reaches _send_can_frame instead of being consumed and
        mis-decoded by the plotting path. `can_id` is the node being configured;
        it's forced to MENU_MODE so param access is accepted by the firmware."""
        self._config_busy.set()
        try:
            # Let the reader notice the flag and finish any in-flight read,
            # then clear stale bytes so we only see our own reply.
            time.sleep(0.02)
            self._clear_serial_buffer()
            self._send_mode_frame(protocol.MENU_MODE, can_id)
            yield
        finally:
            self._config_busy.clear()

    # ── Commands ──────────────────────────────────────────────────────────────
    def send_command(self, can_id, p, v, kp, kd, t_ff):
        """Pack and send a single MIT control command frame."""
        if self.scaling is None:
            raise RuntimeError("Motor parameters not read yet; read them before sending commands")
        command_bytes = protocol.pack_cmd(p=p, v=v, kp=kp, kd=kd, t_ff=t_ff, scaling=self.scaling)
        serial_cmd = protocol.can2serial(can_id, self._hex_bytes(command_bytes))
        self._write(serial_cmd)

    def set_mode(self, mode, can_id):
        self._send_mode_frame(mode, can_id)

    # ── Command streaming ─────────────────────────────────────────────────────
    def update_stream_command(self, can_id, command):
        """Replace the command the streamer repeats. `command` is the kwargs
        dict for send_command(). Cheap and thread-safe (atomic ref swap)."""
        self._stream_cmd = (can_id, command)

    def start_streaming(self, can_id, command):
        """Start repeating `command` every AUTO_SEND_MS from the stream thread.
        Keeps the firmware's CAN-timeout watchdog fed independently of the GUI
        event loop (a dragged window or slow redraw must not stop the motor)."""
        self.update_stream_command(can_id, command)
        self._stream_error = None
        self._stream_enabled.set()

    def stop_streaming(self):
        self._stream_enabled.clear()

    def _stream_loop(self):
        period = AUTO_SEND_MS / 1000.0
        while self.running:
            if not self._stream_enabled.is_set():
                time.sleep(0.02)
                continue
            # A config transaction owns the port (and has forced MENU_MODE);
            # interleaving commands would corrupt its reply capture.
            if self._config_busy.is_set():
                time.sleep(period)
                continue
            cmd = self._stream_cmd
            if cmd is not None:
                try:
                    self.send_command(cmd[0], **cmd[1])
                    if self._stream_error is not None:
                        self._stream_error = None
                        self.status_changed.emit("Command streaming recovered")
                except Exception as e:
                    # Surface the failure (once per distinct error) instead of
                    # silently letting the motor hit its CAN timeout.
                    msg = f"Command stream error: {e}"
                    if msg != self._stream_error:
                        self._stream_error = msg
                        self.status_changed.emit(msg)
            time.sleep(period)

    # ── Config parameter transactions ─────────────────────────────────────────
    @staticmethod
    def _read_payload(index):
        return [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, index]

    @staticmethod
    def _decode_param(reply):
        return None if reply is None else struct.unpack(">f", bytes(reply[2:6]))[0]

    def read_param(self, can_id, index, timeout=0.2):
        """Read a config parameter. Returns its float value, or None on no reply."""
        with self._config_transaction(can_id):
            reply = self._send_can_frame(can_id, self._read_payload(index), timeout=timeout)
        return self._decode_param(reply)

    def read_all_params(self, can_id, indices, timeout=0.2):
        """Read many config parameters within a single config transaction.
        Returns {index: value_or_None}."""
        results = {}
        with self._config_transaction(can_id):
            for index in indices:
                reply = self._send_can_frame(can_id, self._read_payload(index), timeout=timeout)
                results[index] = self._decode_param(reply)
        return results

    def write_param(self, can_id, index, value, timeout=0.2):
        """Write a config parameter. Returns the confirmed value, or None."""
        payload = [0xFF, 0xFF, 0xFD, index] + list(struct.pack(">f", value))
        with self._config_transaction(can_id):
            reply = self._send_can_frame(can_id, payload, timeout=timeout)
        if reply is None:
            return None
        return struct.unpack(">f", bytes(reply[2:6]))[0]

    def reset_params(self, can_id, timeout=0.2):
        """Reset all parameters to defaults. Returns True if a reply was seen."""
        payload = [0xFF, 0xFF, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x00]
        with self._config_transaction(can_id):
            reply = self._send_can_frame(can_id, payload, timeout=timeout)
        return reply is not None

    # ── Background reader ─────────────────────────────────────────────────────
    def _read_loop(self):
        while self.running:
            try:
                # A config transaction owns the port; don't steal its reply.
                # Also idle until scaling is known, since replies can't be
                # decoded without it (and no commands are sent before then).
                if self._config_busy.is_set() or self.scaling is None:
                    time.sleep(0.005)
                    continue

                with self._serial_lock:
                    n = self.serial.in_waiting
                    raw_data = self.serial.read(n) if n else None
                if raw_data is None:
                    # Sleep OUTSIDE the lock; command writes must not wait on us.
                    time.sleep(0.01)
                    continue

                if raw_data and len(raw_data) > 9:
                    can_data = self._extract_can_payload(raw_data)
                    if can_data and len(can_data) >= 8:
                        result = protocol.decode_reply(can_data, scaling=self.scaling)
                        if result:
                            if self._start_time is None:
                                self._start_time = time.time()
                            data = dict(zip(REPLY_FIELDS, result))
                            data['time'] = time.time() - self._start_time
                            self.data_received.emit(data)
                time.sleep(0.1)
            except Exception as e:
                print(f"Read error: {e}")
