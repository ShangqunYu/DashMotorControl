/*
 * user_config.c
 *
 *  Created on: Jul 02, 2026
 *      Author: Simon
 */

#include "user_config.h"
#include <math.h>


#define KT_AFTER_REDUCER 2.97f

void user_config_get_param(param_id_t id, float *out) {
    switch (id) {
        case PARAM_PPAIRS:             *out = PPAIRS;                       return;
        case PARAM_GR:                 *out = GR;                           return;
        case PARAM_KT:                 *out = KT;                           return;
        case PARAM_E_ZERO_RAD:         *out = E_ZERO_RAD;                   return;
        case PARAM_M_ZERO_RAD:         *out = M_ZERO_RAD;                   return;
        case PARAM_I_CAL:              *out = I_CAL;                        return;
        case PARAM_P_MIN:              *out = P_MIN;                        return;
        case PARAM_P_MAX:              *out = P_MAX;                        return;
        case PARAM_V_MIN:              *out = V_MIN;                        return;
        case PARAM_V_MAX:              *out = V_MAX;                        return;
        case PARAM_KP_MAX:             *out = KP_MAX;                       return;
        case PARAM_KD_MAX:             *out = KD_MAX;                       return;
        case PARAM_WINDING_RESISTANCE: *out = R_PHASE;                      return;
        case PARAM_PHASE_ORDER:        *out = (float)PHASE_ORDER;           return;
        case PARAM_CAN_ID:             *out = (float)CAN_ID;                return;
        case PARAM_CAN_TIMEOUT:        *out = (float)CAN_TIMEOUT;           return;
        case PARAM_CALIBRATION_DONE:   *out = (float)CALIBRATION_DONE_FLAG; return;
        default:                       return;
    }
}

void user_config_set_defaults(void) {
    I_BW        = 1000.0f;
    I_MAX       = 60.0f;
    I_FW_MAX    = 0.0f;
    R_NOMINAL   = 0.0f;
    I_MAX_CONT  = 14.0f;
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

    CAN_ID      = 1;
    CAN_MASTER  = 0;
    CAN_TIMEOUT = 10000;
    E_ZERO      = 0;
    M_ZERO      = 0;
}

void user_config_apply_defaults(void) {
    if (isnan(I_BW)       || I_BW       == -1) I_BW       = 1000.0f;
    if (isnan(I_MAX)      || I_MAX      == -1) I_MAX      = 60.0f;
    if (isnan(I_FW_MAX)   || I_FW_MAX   == -1) I_FW_MAX   = 0.0f;
    if (isnan(R_NOMINAL)  || R_NOMINAL  == -1) R_NOMINAL  = 0.0f;
    if (isnan(I_MAX_CONT) || I_MAX_CONT == -1) I_MAX_CONT = 14.0f;
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
    if (E_ZERO      == -1) E_ZERO      = 0;
    if (M_ZERO      == -1) M_ZERO      = 0;
}
