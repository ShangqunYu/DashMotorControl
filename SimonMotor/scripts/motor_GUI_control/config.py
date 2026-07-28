"""Static configuration for the motor-control GUI: tunable parameters, plot
channels, and the serial/timing defaults. Kept free of Qt so it can be imported
anywhere (including tests) without a running application."""

# ── Serial / connection defaults ──────────────────────────────────────────────
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 921600

# Default CAN IDs used by the control and config panels.
DEFAULT_COMMAND_CAN_ID = "1200FD01"
DEFAULT_CONFIG_CAN_ID = "00000001"

# ── Timing ────────────────────────────────────────────────────────────────────
PLOT_REFRESH_MS = 100      # how often the plots redraw (decoupled from serial)
AUTO_SEND_MS = 5           # command poll interval while in MIT mode
PLOT_WINDOW_SECONDS = 10.0  # rolling history shown on the plots

# ── Configurable motor parameters (index, name, unit, default) ────────────────
# Indices with bit 0x80 set are integer-valued; the rest are IEEE floats.
CONFIG_PARAMS = [
    (0, "Pole pairs", "", 21.0),
    (1, "Gear ratio", "", 18.0),
    (2, "Torque constant", "N·m/A", 2.97 / 18.0),
    (3, "Electrical zero offset", "rad", 0.0),
    (4, "Mechanical zero offset", "rad", 0.0),
    (5, "Current limit", "A", 60.0),
    (6, "Position min", "rad", -12.57),
    (7, "Position max", "rad", 12.57),
    (8, "Velocity min", "rad/s", -65.0),
    (9, "Velocity max", "rad/s", 65.0),
    (10, "Max position gain", "kP", 500.0),
    (11, "Max velocity gain", "kD", 5.0),
    (12, "Calibration current", "A", 10.0),
    (13, "D/Q position gain", "kp on d/q axes", 0.05),
    (14, "D/Q integral gain", "ki on d/q axes", 200.0),
    (15, "Temp min", "°C", -40.0),
    (16, "Temp max", "°C", 180.0),
    (17, "Resistance", "Ω", 0.05),
    (18, "Inductance", "H", 0.00001),
    (19, "Thermistor beta", "K", 3950.0),
    (20, "Temp cutoff", "°C overtemp safety trip", 100.0),
    (0x80 | 0, "Encoder select", "0=internal,1=external", 0),
    (0x80 | 1, "Phase order", "0 same order as encoder, 1 reversed", 0),
    (0x80 | 2, "CAN ID", "node id (dec)", 1),
    (0x80 | 3, "CAN master", "", 0),
    (0x80 | 4, "CAN timeout", "FOC cycles of 25 µs; 10000 = 250 ms", 10000),
    (0x80 | 5, "Calibration done", "", 0),
]

# Convenience lookup: parameter index -> default value.
CONFIG_DEFAULTS = {index: value for index, _, _, value in CONFIG_PARAMS}

# Config indices whose values define the MIT command/reply scaling ranges.
# Commands can't be sent safely until all of these have been read off the motor.
#   1 gear ratio, 2 torque constant, 5 current limit -> torque range;
#   6/7 position, 8/9 velocity, 10 kP max, 11 kD max, 15/16 temperature range.
SCALING_PARAM_INDICES = (1, 2, 5, 6, 7, 8, 9, 10, 11, 15, 16)

# ── Live plot channels ────────────────────────────────────────────────────────
# `key` order must match the tuple returned by protocol.decode_reply().
# si_prefix=False keeps axes in base units (e.g. "rad" instead of "mrad").
PLOT_CHANNELS = [
    {"key": "position", "title": "Position (rad)",   "label": "Position",    "units": "rad",   "color": "cyan",    "row": 0, "col": 0, "colspan": 1, "si_prefix": False},
    {"key": "velocity", "title": "Velocity (rad/s)", "label": "Velocity",    "units": "rad/s", "color": "lime",    "row": 0, "col": 1, "colspan": 1, "si_prefix": False},
    {"key": "torque",   "title": "Torque (N-m)",     "label": "Torque",      "units": "N-m",   "color": "yellow",  "row": 1, "col": 0, "colspan": 1, "si_prefix": True},
    {"key": "vbus",     "title": "Bus Voltage (V)",  "label": "Voltage",     "units": "V",     "color": "magenta", "row": 1, "col": 1, "colspan": 1, "si_prefix": True},
    {"key": "temp",     "title": "Temperature (°C)", "label": "Temperature", "units": "°C",    "color": "red",     "row": 2, "col": 0, "colspan": 2, "si_prefix": True},
]

# Field order of a decoded status reply (matches PLOT_CHANNELS order).
REPLY_FIELDS = tuple(ch["key"] for ch in PLOT_CHANNELS)
