/*
 * user_config.h
 *
 *  Created on: Jan 22, 2026
 *      Author: simon
 */

#ifndef INC_USER_CONFIG_H_
#define INC_USER_CONFIG_H_

#include <stdbool.h>

/* Sizes of the two register banks. Single-sourced here so main.c's definitions
 * and the bounds checks in user_config.c can never drift apart. */
#define FLOAT_REG_SIZE 64
#define INT_REG_SIZE   256


#define CFG_PPAIRS           __float_reg[0]              // Number of motor pole-pairs
#define CFG_GR               __float_reg[1]              // Gear ratio
#define CFG_KT               __float_reg[2]              // Torque constant
#define CFG_E_ZERO_RAD       __float_reg[3]              // Electrical zero offset
#define CFG_M_ZERO_RAD       __float_reg[4]              // Mechanical zero offset
#define CFG_I_MAX            __float_reg[5]              // Current limit
#define CFG_P_MIN            __float_reg[6]              // Position setpoint lower limit
#define CFG_P_MAX            __float_reg[7]              // Position setpoint upper limit
#define CFG_V_MIN            __float_reg[8]              // Velocity setpoint lower limit
#define CFG_V_MAX            __float_reg[9]              // Velocity setpoint upper limit
#define CFG_KP_MAX           __float_reg[10]              // Max position gain
#define CFG_KD_MAX           __float_reg[11]              // Max velocity gain
#define CFG_I_CAL            __float_reg[12]              // Calibration current
#define CFG_KP_DQ            __float_reg[13]              // D/Q axis position gain
#define CFG_KI_DQ            __float_reg[14]              // D/Q axis integral gain
#define CFG_TEMP_MIN         __float_reg[15]              // Temperature safety limit
#define CFG_TEMP_MAX         __float_reg[16]              // Temperature safety limit
#define CFG_RESISTANCE      __float_reg[17]              // D/Q axis winding resistance
#define CFG_INDUCTANCE      __float_reg[18]              // D/Q axis winding inductance
#define CFG_THERMISTOR_BETA __float_reg[19]              // NTC beta coefficient (K), varies by thermistor variant


#define CFG_ENC_SEL                __int_reg[0]
#define CFG_PHASE_ORDER            __int_reg[1]
#define CFG_CAN_ID                 __int_reg[2]
#define CFG_CAN_MASTER             __int_reg[3]
#define CFG_CAN_TIMEOUT            __int_reg[4]
#define CFG_CALIBRATION_DONE_FLAG  __int_reg[5]
#define CFG_ENCODER_LUT            __int_reg[7]


extern float __float_reg[];
extern int __int_reg[];

/* param_id_t encodes the storage location directly:
 *   bits[6:0] = array index
 *   bit[7]  = 0 → __float_reg,  1 → __int_reg
 * Adding a new readable param is one line here; no switch needed in get_param. */
typedef enum {
   PARAM_CFG_PPAIRS          = 0,         // __float_reg[0]
   PARAM_CFG_GR              = 1,         // __float_reg[1]
   PARAM_CFG_KT              = 2,         // __float_reg[2]
   PARAM_CFG_E_ZERO_RAD      = 3,         // __float_reg[3]
   PARAM_CFG_M_ZERO_RAD      = 4,         // __float_reg[4]
   PARAM_CFG_I_MAX           = 5,         // __float_reg[5]
   PARAM_CFG_P_MIN           = 6,         // __float_reg[6]
   PARAM_CFG_P_MAX           = 7,         // __float_reg[7]
   PARAM_CFG_V_MIN           = 8,         // __float_reg[8]
   PARAM_CFG_V_MAX           = 9,         // __float_reg[9]
   PARAM_CFG_KP_MAX          = 10,        // __float_reg[10]
   PARAM_CFG_KD_MAX          = 11,        // __float_reg[11]
   PARAM_CFG_I_CAL           = 12,        // __float_reg[12]
   PARAM_CFG_KP_DQ           = 13,        // __float_reg[13]
   PARAM_CFG_KI_DQ           = 14,        // __float_reg[14]
   PARAM_CFG_TEMP_MIN        = 15,        // __float_reg[15]
   PARAM_CFG_TEMP_MAX        = 16,        // __float_reg[16]
   PARAM_CFG_RESISTANCE      = 17,        // __float_reg[17]
   PARAM_CFG_INDUCTANCE      = 18,        // __float_reg[18]
   PARAM_CFG_THERMISTOR_BETA = 19,        // __float_reg[19]

   PARAM_CFG_ENC_SEL               = 0x80 | 0,  // __int_reg[0]
   PARAM_CFG_PHASE_ORDER           = 0x80 | 1,  // __int_reg[1]
   PARAM_CFG_CAN_ID                = 0x80 | 2,  // __int_reg[2]
   PARAM_CFG_CAN_MASTER            = 0x80 | 3,  // __int_reg[3]
   PARAM_CFG_CAN_TIMEOUT           = 0x80 | 4,  // __int_reg[4]
   PARAM_CFG_CALIBRATION_DONE_FLAG = 0x80 | 5,  // __int_reg[5]
   PARAM_CFG_ENCODER_LUT           = 0x80 | 7,  // __int_reg[7]  (128-element array)
} param_id_t;

/* True if `id` maps to an in-bounds slot in its register bank. Guards against
 * out-of-range indices arriving from the CAN bus. */
bool user_config_param_valid(param_id_t id);

/* Returns the parameter value as a float (int params are cast). Invalid ids
 * yield 0.0f. */
void user_config_get_param(param_id_t id, float *out);

/* Writes a parameter. Float params written directly; int params cast from float.
 * Invalid ids are ignored. */
void user_config_set_param(param_id_t id, float value);

/* Apply defaults only to fields that are uninitialized (erased flash reads as -1/NaN).
   Call this once after preference_writer_load() on boot. */
void user_config_apply_defaults(void);

/* Unconditionally reset every parameter to its default value.
   Call this for a factory reset, then open/flush/close the preference_writer. */
void user_config_set_defaults(void);


#endif /* INC_USER_CONFIG_H_ */
