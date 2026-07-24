import socket
import struct
import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont

UDP_IP = "172.31.201.50"
UDP_PORT = 8888

MENU_MODE = 0
MIT_MODE = 5

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

root = tk.Tk()
root.title("UDP Command Sender")

root.geometry("600x400") # Initial size
root.minsize(750, 400)

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

default_font = tkfont.Font(family="Helvetica", size=14, weight="bold")

'''
style = ttk.Style()
style.theme_use("alt")
style.configure('TButton', background="#6FC8E9", foreground='white', font=('Helvetica', 12))
style.map('TButton', background=[('active', "#1f9dc4")])'''

amp = tk.DoubleVar(value=0.6)
freq = tk.DoubleVar(value=1.0)
kp = tk.DoubleVar(value=40.0)
kd = tk.DoubleVar(value=0.5)
mode = tk.BooleanVar(value=False)
can_bus_no = tk.IntVar(value=0)
can_id = tk.IntVar(value=1)

def add_slider(label, var, frm, to, resolution):
    frame = tk.Frame(root)
    frame.pack(fill="x", padx=10, pady=5)

    tk.Label(frame, text=label, width=10, anchor="w", font=default_font).pack(side="left")

    value = tk.Label(frame, width=8, font=default_font)
    value.pack(side="right")

    def update(v):
        value.configure(text=f"{float(v):.3f}")

    scale = tk.Scale(
        frame,
        variable=var,
        from_=frm,
        to=to,
        resolution=resolution,
        orient=tk.HORIZONTAL,
        command=update,
        showvalue=0,
        length=350
    )
    scale.pack(fill="x", expand=True)

    update(var.get())

add_slider("Amplitude", amp, 0.0, 2.0, 0.1)
add_slider("Frequency", freq, 0.0, 5.0, 0.1)
add_slider("Kp", kp, 0.0, 60.0, 1.0)
add_slider("Kd", kd, 0.0, 5.0, 0.1)
add_slider("CAN bus", can_bus_no, 0, 3, 1)
add_slider("CAN ID", can_id, 0, 5, 1)

tk.Checkbutton(root, text="MIT Mode", variable=mode, font=default_font).pack(pady=10)

status = tk.Label(root, text="", font=default_font)
status.pack()

def send_packet():
    packet = struct.pack(
        "<ffffIII", # 4 floats + 3 uint32
        amp.get(), # amplitude
        freq.get(), # frequency
        kp.get(), # kp
        kd.get(), # kd
        MIT_MODE if mode.get() else MENU_MODE, # mode
        can_bus_no.get(), # CAN bus
        can_id.get() # CAN ID
    )

    sock.sendto(packet, (UDP_IP, UDP_PORT))

    status.config(
        text=f"Sent: A={amp.get():.2f}  "
             f"F={freq.get():.2f}  "
             f"Kp={kp.get():.1f}  "
             f"Kd={kd.get():.2f}  "
             f"Mode={'MIT' if mode.get() else 'MENU'}  "
             f"CAN bus={can_bus_no.get()}  "
             f"CAN ID={can_id.get()}"
    )

def on_enter(e):
    send_button.config(relief="sunken")
def on_leave(e):
    send_button.config(relief="raised")

send_button = tk.Button(root, text="Send Command", command=send_packet, bg="lightblue", font=default_font)
send_button.bind("<Enter>", on_enter)
send_button.bind("<Leave>", on_leave)
send_button.pack(pady=15)

root.mainloop()