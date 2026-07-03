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
    I_MAX       = 60.0f;
    I_CAL       = 10.0f;
    PPAIRS      = 21.0f;
    GR          = 18.0f;
    KT          = KT_AFTER_REDUCER / GR;
    KP_MAX      = 500.0f;
    KD_MAX      = 5.0f;
    P_MAX       = 12.57f;
    P_MIN       = -12.57f;
    V_MAX       = 65.0f;
    V_MIN       = -65.0f;
    E_ZERO_RAD  = 0.0f;
    M_ZERO_RAD  = 0.0f;
    memset(&__int_reg[7], 0, 128 * sizeof(int));  /* ENCODER_LUT */

    CAN_ID      = 1;
    CAN_MASTER  = 0;
    CAN_TIMEOUT = 10000;
    CALIBRATION_DONE_FLAG = 0;
    
}

void user_config_apply_defaults(void) {
    if (isnan(I_MAX)      || I_MAX      == -1) I_MAX      = 60.0f;
    if (isnan(I_CAL)      || I_CAL      == -1) I_CAL      = 10.0f;
    if (isnan(PPAIRS)     || PPAIRS     == -1) PPAIRS     = 21.0f;
    if (isnan(GR)         || GR         == -1) GR         = 18.0f;
    if (isnan(KT)         || KT         == -1) KT         = KT_AFTER_REDUCER / GR;
    if (isnan(KP_MAX)     || KP_MAX     == -1) KP_MAX     = 500.0f;
    if (isnan(KD_MAX)     || KD_MAX     == -1) KD_MAX     = 5.0f;
    if (isnan(P_MAX))                          P_MAX      = 12.57f;
    if (isnan(P_MIN))                          P_MIN      = -12.57f;
    if (isnan(V_MAX))                          V_MAX      = 65.0f;
    if (isnan(V_MIN))                          V_MIN      = -65.0f;
    if (isnan(E_ZERO_RAD))                     E_ZERO_RAD = 0.0f;
    if (isnan(M_ZERO_RAD))                     M_ZERO_RAD = 0.0f;

    if (CAN_ID      == -1) CAN_ID      = 1;
    if (CAN_MASTER  == -1) CAN_MASTER  = 0;
    if (CAN_TIMEOUT == -1) CAN_TIMEOUT = 10000;
}
