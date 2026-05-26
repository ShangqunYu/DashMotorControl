#!/usr/bin/env python3
"""
CAN Frame Sender and Receiver with GUI
Sends CAN frames every 20ms and displays all incoming CAN frames
"""

import can
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime


class CANInterface:
    def __init__(self, channel="can0"):
        self.bus = None
        self.channel = channel
        self.running = False
        self.receive_callback = None
        self.receive_thread = None
        
    def connect(self):
        """Connect to the CAN bus"""
        try:
            self.bus = can.interface.Bus(channel=self.channel, interface="socketcan")
            self.running = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the CAN bus"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1)
        if self.bus:
            self.bus.shutdown()
    
    def send_message(self, can_id, data, is_extended=False):
        """Send a CAN message"""
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=is_extended
            )
            self.bus.send(msg)
            return True, msg
        except Exception as e:
            return False, str(e)
    
    def start_receiving(self, callback):
        """Start receiving CAN messages in a separate thread"""
        self.receive_callback = callback
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
    
    def _receive_loop(self):
        """Continuously receive CAN messages"""
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg and self.receive_callback:
                    self.receive_callback(msg)
            except Exception as e:
                pass


class CANSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CAN Frame Sender & Receiver")
        self.root.geometry("800x700")
        
        self.can_interface = CANInterface()
        self.send_thread = None
        self.sending = False
        self.send_interval = 20  # milliseconds
        
        self._create_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
    
    def _create_widgets(self):
        """Create the GUI widgets"""
        
        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="CAN Interface:").pack(side=tk.LEFT, padx=5)
        self.interface_var = tk.StringVar(value="can0")
        interface_entry = ttk.Entry(conn_frame, textvariable=self.interface_var, width=15)
        interface_entry.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._connect)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self._disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # --- Send Frame ---
        send_frame = ttk.LabelFrame(self.root, text="Send CAN Frame", padding=10)
        send_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(send_frame, text="CAN ID (hex):").pack(side=tk.LEFT, padx=5)
        self.can_id_var = tk.StringVar(value="0x123")
        can_id_entry = ttk.Entry(send_frame, textvariable=self.can_id_var, width=15)
        can_id_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(send_frame, text="Data (hex, space-separated):").pack(side=tk.LEFT, padx=5)
        self.data_var = tk.StringVar(value="11 22 33 44")
        data_entry = ttk.Entry(send_frame, textvariable=self.data_var, width=30)
        data_entry.pack(side=tk.LEFT, padx=5)
        
        self.is_extended_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(send_frame, text="Extended ID", variable=self.is_extended_var).pack(side=tk.LEFT, padx=5)
        
        # --- Send Options ---
        options_frame = ttk.LabelFrame(self.root, text="Send Options", padding=10)
        options_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(options_frame, text="Interval (ms):").pack(side=tk.LEFT, padx=5)
        self.interval_var = tk.StringVar(value="20")
        interval_entry = ttk.Entry(options_frame, textvariable=self.interval_var, width=10)
        interval_entry.pack(side=tk.LEFT, padx=5)
        
        self.send_once_btn = ttk.Button(options_frame, text="Send Once", command=self._send_once, state=tk.DISABLED)
        self.send_once_btn.pack(side=tk.LEFT, padx=5)
        
        self.send_repeat_btn = ttk.Button(options_frame, text="Start Sending", command=self._start_sending, state=tk.DISABLED)
        self.send_repeat_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_send_btn = ttk.Button(options_frame, text="Stop Sending", command=self._stop_sending, state=tk.DISABLED)
        self.stop_send_btn.pack(side=tk.LEFT, padx=5)
        
        self.send_status_label = ttk.Label(options_frame, text="Status: Idle", foreground="blue")
        self.send_status_label.pack(side=tk.LEFT, padx=20)
        
        # --- Receive Frame ---
        recv_frame = ttk.LabelFrame(self.root, text="Received CAN Frames", padding=10)
        recv_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create text widget for displaying received frames
        self.recv_text = scrolledtext.ScrolledText(recv_frame, height=15, width=90, state=tk.DISABLED)
        self.recv_text.pack(fill=tk.BOTH, expand=True)
        
        # Configure tags for colored output
        self.recv_text.tag_config("timestamp", foreground="gray")
        self.recv_text.tag_config("id", foreground="blue", font=("Arial", 10, "bold"))
        self.recv_text.tag_config("data", foreground="green")
        
        # Clear button
        clear_btn = ttk.Button(recv_frame, text="Clear", command=self._clear_receive)
        clear_btn.pack(side=tk.RIGHT, padx=5, pady=5)
    
    def _connect(self):
        """Connect to the CAN bus"""
        interface = self.interface_var.get()
        if self.can_interface.connect():
            self.status_label.config(text=f"Status: Connected to {interface}", foreground="green")
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.send_once_btn.config(state=tk.NORMAL)
            self.send_repeat_btn.config(state=tk.NORMAL)
            self.can_interface.start_receiving(self._on_message_received)
            self._log_receive("Connected to CAN interface\n")
        else:
            messagebox.showerror("Connection Error", f"Failed to connect to {interface}")
    
    def _disconnect(self):
        """Disconnect from the CAN bus"""
        self._stop_sending()
        self.can_interface.disconnect()
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.send_once_btn.config(state=tk.DISABLED)
        self.send_repeat_btn.config(state=tk.DISABLED)
        self._log_receive("Disconnected from CAN interface\n")
    
    def _send_once(self):
        """Send a CAN frame once"""
        try:
            can_id = int(self.can_id_var.get(), 0)
            data_str = self.data_var.get().strip()
            data = [int(x, 0) for x in data_str.split()]
            is_extended = self.is_extended_var.get()
            
            if len(data) > 8:
                messagebox.showerror("Error", "CAN data cannot exceed 8 bytes")
                return

            if any(byte < 0 or byte > 0xFF for byte in data):
                messagebox.showerror("Error", "Each data byte must be between 0x00 and 0xFF")
                return

            max_can_id = 0x1FFFFFFF if is_extended else 0x7FF
            frame_type = "extended" if is_extended else "standard"
            if can_id < 0 or can_id > max_can_id:
                messagebox.showerror(
                    "Error",
                    f"Invalid {frame_type} CAN ID. Expected range: 0x0 to 0x{max_can_id:X}"
                )
                return
            
            success, result = self.can_interface.send_message(can_id, data, is_extended=is_extended)
            if success:
                self._log_send(f"Sent: {result}")
            else:
                messagebox.showerror("Send Error", f"Failed to send: {result}")
        except ValueError:
            messagebox.showerror("Input Error", "Invalid CAN ID or data format")
    
    def _start_sending(self):
        """Start sending CAN frames periodically"""
        try:
            self.send_interval = int(self.interval_var.get())
            if self.send_interval < 1:
                messagebox.showerror("Error", "Interval must be >= 1 ms")
                return
        except ValueError:
            messagebox.showerror("Input Error", "Invalid interval format")
            return
        
        self.sending = True
        self.send_repeat_btn.config(state=tk.DISABLED)
        self.stop_send_btn.config(state=tk.NORMAL)
        self.send_once_btn.config(state=tk.DISABLED)
        self.send_status_label.config(text=f"Status: Sending every {self.send_interval}ms", foreground="orange")
        
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
    
    def _stop_sending(self):
        """Stop sending CAN frames"""
        self.sending = False
        if self.send_thread:
            self.send_thread.join(timeout=1)
        
        self.send_repeat_btn.config(state=tk.NORMAL)
        self.stop_send_btn.config(state=tk.DISABLED)
        self.send_once_btn.config(state=tk.NORMAL)
        self.send_status_label.config(text="Status: Idle", foreground="blue")
    
    def _send_loop(self):
        """Continuously send CAN frames"""
        import time
        while self.sending and self.can_interface.running:
            self._send_once()
            time.sleep(self.send_interval / 1000.0)
    
    def _on_message_received(self, msg):
        """Callback for received CAN messages"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        can_id = f"0x{msg.arbitration_id:X}"
        frame_type = "EXT" if msg.is_extended_id else "STD"
        data_str = " ".join(f"{b:02X}" for b in msg.data)
        
        message = f"[{timestamp}] {frame_type} ID: {can_id} | Data: {data_str}\n"
        self._log_receive(message)
    
    def _log_send(self, message):
        """Log sent message"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] {message}")
    
    def _log_receive(self, message):
        """Log received message to the text widget"""
        self.recv_text.config(state=tk.NORMAL)
        self.recv_text.insert(tk.END, message)
        self.recv_text.see(tk.END)
        self.recv_text.config(state=tk.DISABLED)
    
    def _clear_receive(self):
        """Clear the receive log"""
        self.recv_text.config(state=tk.NORMAL)
        self.recv_text.delete(1.0, tk.END)
        self.recv_text.config(state=tk.DISABLED)
    
    def _on_closing(self):
        """Handle window closing"""
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = CANSenderApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
