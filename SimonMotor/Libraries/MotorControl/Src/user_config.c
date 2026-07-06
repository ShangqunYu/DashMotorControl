/*
 * user_config.c
 *
 *  Created on: Jul 02, 2026
 *      Author: Simon
 */

#include "user_config.h"
#include <math.h>
#include <stdint.h>
#include <string.h>


#define KT_AFTER_REDUCER 2.97f

void user_config_get_param(param_id_t id, float *out) {
    uint8_t idx = (uint8_t)id;
    if (idx & 0x80) {
        *out = (float)__int_reg[idx & 0x7F];
    } else {
        *out = __float_reg[idx];
    }
}

void user_config_set_param(param_id_t id, float value) {
    uint8_t idx = (uint8_t)id;
    if (idx & 0x80) {
        __int_reg[idx & 0x7F] = (int)value;
    } else {
        __float_reg[idx] = value;
    }
}

void user_config_set_defaults(void) {
    CFG_PPAIRS                              = 21.0f;
    CFG_GR                                  = 18.0f;
    CFG_KT                                  = KT_AFTER_REDUCER / CFG_GR;
    CFG_E_ZERO_RAD                          =   0.0f;
    CFG_M_ZERO_RAD                          =   0.0f;
    CFG_I_MAX                               =  60.0f;
    CFG_P_MIN                               = -12.57f;
    CFG_P_MAX                               =  12.57f;
    CFG_V_MAX                               = 65.0f;
    CFG_V_MIN                               = -65.0f;
    CFG_KP_MAX                              = 500.0f;
    CFG_KD_MAX                              = 5.0f;
    CFG_I_CAL                               = 10.0f;
    CFG_KP_DQ                               = 0.05f;
    CFG_KI_DQ                               = 200.0f;
    CFG_TEMP_MIN                            = -40.0f;
    CFG_TEMP_MAX                            = 180.0f;
    CFG_RESISTANCE                          = 0.05f;
    CFG_INDUCTANCE                          = 0.00001f;
    CFG_ENC_SEL                             = 0;  /* 1 = external encoder */
    CFG_PHASE_ORDER                         = 0;  /* 1 = reverse phase order */
    CFG_CAN_ID                              = 1;
    CFG_CAN_MASTER                          = 0;
    CFG_CAN_TIMEOUT                         = 10000;
    CFG_CALIBRATION_DONE_FLAG               = 0;

    memset(&__int_reg[7], 0, 128 * sizeof(int));  /* CFG_ENCODER_LUT */
    
}

void user_config_apply_defaults(void) {
    if (isnan(CFG_I_MAX)      || CFG_I_MAX      == -1) CFG_I_MAX      = 60.0f;
    if (isnan(CFG_I_CAL)      || CFG_I_CAL      == -1) CFG_I_CAL      = 10.0f;
    if (isnan(CFG_PPAIRS)     || CFG_PPAIRS     == -1) CFG_PPAIRS     = 21.0f;
    if (isnan(CFG_TEMP_MAX)   || CFG_TEMP_MAX  == -1)  CFG_TEMP_MAX  = 180.0f;
    if (isnan(CFG_GR)         || CFG_GR         == -1) CFG_GR         = 18.0f;
    if (isnan(CFG_KT)         || CFG_KT         == -1) CFG_KT         = KT_AFTER_REDUCER / CFG_GR;
    if (isnan(CFG_KP_MAX)     || CFG_KP_MAX     == -1) CFG_KP_MAX     = 500.0f;
    if (isnan(CFG_KD_MAX)     || CFG_KD_MAX     == -1) CFG_KD_MAX     = 5.0f;
    if (isnan(CFG_P_MAX))                          CFG_P_MAX      = 12.57f;
    if (isnan(CFG_P_MIN))                          CFG_P_MIN      = -12.57f;
    if (isnan(CFG_V_MAX))                          CFG_V_MAX      = 65.0f;
    if (isnan(CFG_V_MIN))                          CFG_V_MIN      = -65.0f;
    if (isnan(CFG_E_ZERO_RAD))                     CFG_E_ZERO_RAD = 0.0f;
    if (isnan(CFG_M_ZERO_RAD))                     CFG_M_ZERO_RAD = 0.0f;

    if (CFG_ENC_SEL     == -1) CFG_ENC_SEL     = 1;
    if (CFG_CAN_ID      == -1) CFG_CAN_ID      = 1;
    if (CFG_CAN_MASTER  == -1) CFG_CAN_MASTER  = 0;
    if (CFG_CAN_TIMEOUT == -1) CFG_CAN_TIMEOUT = 10000;
}
