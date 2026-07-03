/*
 * user_config.h
 *
 *  Created on: Jan 22, 2026
 *      Author: simon
 */

#ifndef INC_USER_CONFIG_H_
#define INC_USER_CONFIG_H_



/*
PPAIRS
GR
KT
E_ZERO_RAD
M_ZERO_RAD
I_CAL
P_MIN
P_MAX
V_MIN
V_MAX
KP_MAX
KD_MAX
WINDING_RESISTANCE
WINDING_INDUCTANCE

PHASE_ORDER
CAN_ID
CAN_TIMEOUT
CALIBRATION_DONE_FLAG
*/


#define I_MAX                    __float_reg[3]                            // Current limit
#define TEMP_MAX                 __float_reg[8]                            // Temperature safety lmit
#define PPAIRS					      __float_reg[10]									// Number of motor pole-pairs
#define R_PHASE					   __float_reg[13]									// Single phase resistance
#define KT						      __float_reg[14]									// Torque Constant for the rotor (N-m/A) (note: this is the torque constant before the gear reducer, so the effective torque constant at the output shaft is KT*GR)
#define GR						      __float_reg[17]									// Gear ratio
#define I_CAL					      __float_reg[18]									// Calibration Current
#define P_MIN					      __float_reg[19]									// Position setpoint lower limit (rad)
#define P_MAX					      __float_reg[20]									// Position setupoint upper bound (rad)
#define V_MIN					      __float_reg[21]									// Velocity setpoint lower bound (rad/s)
#define V_MAX					      __float_reg[22]									// Velocity setpoint upper bound (rad/s)
#define KP_MAX					      __float_reg[23]									// Max position gain (N-m/rad)
#define KD_MAX					      __float_reg[24]									// Max velocity gain (N-m/rad/s)
#define E_ZERO_RAD				   __float_reg[25]									// Electrical zero offset (rad), saved after encoder calibration
#define M_ZERO_RAD				   __float_reg[26]									// Mechanical zero (rad), set by user via SET_ZERO_MODE


#define PHASE_ORDER              __int_reg[0]                                    // Phase swapping during calibration
#define CAN_ID                   __int_reg[1]                                    // CAN bus ID
#define CAN_MASTER               __int_reg[2]                                    // CAN bus "master" ID
#define CAN_TIMEOUT              __int_reg[3]                                    // CAN bus timeout period
#define CALIBRATION_DONE_FLAG	   __int_reg[6]									// Set to 1 when calibration finishes, can be used for external scripts to detect when it's safe to read the LUT values
#define ENCODER_LUT              __int_reg[7]                                    // Encoder offset LUT - 128 elements long




extern float __float_reg[];
extern int __int_reg[];

/* param_id_t encodes the storage location directly:
 *   bits[6:0] = array index
 *   bit[7]  = 0 → __float_reg,  1 → __int_reg
 * Adding a new readable param is one line here; no switch needed in get_param. */
typedef enum {
    PARAM_PPAIRS             = 10,
    PARAM_WINDING_RESISTANCE = 13,        // R_PHASE
    PARAM_KT                 = 14,
    PARAM_GR                 = 17,
    PARAM_I_CAL              = 18,
    PARAM_P_MIN              = 19,
    PARAM_P_MAX              = 20,
    PARAM_V_MIN              = 21,
    PARAM_V_MAX              = 22,
    PARAM_KP_MAX             = 23,
    PARAM_KD_MAX             = 24,
    PARAM_E_ZERO_RAD         = 25,
    PARAM_M_ZERO_RAD         = 26,

    PARAM_PHASE_ORDER        = 0x80 | 0,  // __int_reg[0]
    PARAM_CAN_ID             = 0x80 | 1,  // __int_reg[1]
    PARAM_CAN_TIMEOUT        = 0x80 | 3,  // __int_reg[3]
    PARAM_CALIBRATION_DONE   = 0x80 | 6,  // __int_reg[6]
} param_id_t;

/* Returns the parameter value as a float (int params are cast). */
void user_config_get_param(param_id_t id, float *out);

/* Writes a parameter. Float params written directly; int params cast from float. */
void user_config_set_param(param_id_t id, float value);

/* Apply defaults only to fields that are uninitialized (erased flash reads as -1/NaN).
   Call this once after preference_writer_load() on boot. */
void user_config_apply_defaults(void);

/* Unconditionally reset every parameter to its default value.
   Call this for a factory reset, then open/flush/close the preference_writer. */
void user_config_set_defaults(void);


#endif /* INC_USER_CONFIG_H_ */
