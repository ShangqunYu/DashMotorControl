
# ── Mode bytes (must match foc.h) ────────────────────────────────────────────
MENU_MODE = 0
CALIBRATION_MODE = 1
MIT_MODE  = 2

# ── Fixed hardware ranges (hw_config.h; not user-configurable) ───────────────
# Bus-voltage reply is scaled to these on the motor; unlike the temperature
# range they are compile-time constants, so they stay here rather than in the
# read-from-flash MotorScaling.
V_BUS_MIN = 0.0    # V
V_BUS_MAX = 60.0   # V

def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))

def uint_to_float(x, x_min, x_max, bits):
    return x * (x_max - x_min) / ((1 << bits) - 1) + x_min

def pack_cmd(p, v, kp, kd, t_ff, scaling):
    # `scaling` (a MotorScaling) carries the ranges read from the motor's flash.
    p_int  = float_to_uint(p,    scaling.p_min, scaling.p_max,  16)
    v_int  = float_to_uint(v,    scaling.v_min, scaling.v_max,  12)
    kp_int = float_to_uint(kp,   0,             scaling.kp_max, 12)
    kd_int = float_to_uint(kd,   0,             scaling.kd_max, 12)
    t_int  = float_to_uint(t_ff, scaling.t_min, scaling.t_max,  12)
    return [
        p_int >> 8,
        p_int & 0xFF,
        v_int >> 4,
        ((v_int & 0xF) << 4) | (kp_int >> 8),
        kp_int & 0xFF,
        kd_int >> 4,
        ((kd_int & 0xF) << 4) | (t_int >> 8),
        t_int & 0xFF,
    ]

def decode_reply(d, scaling):
    if len(d) < 8:
        return None

    # `scaling` carries the ranges read from the motor's flash.
    p_int = (d[1] << 8) | d[2]
    v_int = (d[3] << 4) | (d[4] >> 4)
    t_int = ((d[4] & 0xF) << 8) | d[5]


    return (
        uint_to_float(p_int, scaling.p_min, scaling.p_max, 16),
        uint_to_float(v_int, scaling.v_min, scaling.v_max, 12),
        uint_to_float(t_int, scaling.t_min, scaling.t_max, 12),
        uint_to_float(d[6], V_BUS_MIN, V_BUS_MAX, 8),
        uint_to_float(d[7], scaling.temp_min, scaling.temp_max, 8),
    )

def can2serial(can_id:str, data:str, printing:bool = False)->bytes:
    '''
        can_id: hexadecimal string. 8-digit.
        data: hexadecimal string. 16-digit.
        output: hexadecimal string. 34-digit. 4+8+2+16+4
    '''

    parsed_can_id = parse_hex(can_id)
    parsed_data = parse_hex(data)

    if len(parsed_can_id) != 8:
            raise ValueError(f"can_id must be 8-digit.")
    if len(parsed_data) != 16:
        raise ValueError(f"data must be 16-digit.")
    
    can_id_int = int(parsed_can_id, 16)
    data_int = int(parsed_data, 16)

    frame_header = 0x4154
    frame_tail = 0x0d0a
    extended_frame = 0x08
    dummy_bit = 0b100

    can_id_restructured = can_id_int<<3 | dummy_bit

    output = f"{frame_header:04x}{can_id_restructured:08x}{extended_frame:02x}{data_int:016x}{frame_tail:04x}"

    if printing:
        pretty_output = " ".join(output[i:i+2] for i in range(0, len(output), 2))
        print(f"Output Serial: {pretty_output.lower()}")

    return bytes.fromhex(output)

def parse_hex(hex_input: str) -> str:
    if not isinstance(hex_input, str):
        raise TypeError("Input must be string.")
    
    cleaned = hex_input.replace(" ", "")
    if cleaned.lower().startswith("0x"):
        cleaned = cleaned[2:]
        
    if not cleaned:
        raise ValueError("Invalid Input.")
        
    return cleaned