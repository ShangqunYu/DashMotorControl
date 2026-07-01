/*
 * foc.c
 *
 *  Created on: Jul 01, 2026
 *      Author: Simon
 */

#include "foc.h"
#include "math_utils.h"
#include <math.h>



void abc(float sf, float cf, float d, float q, float *a, float *b, float *c) {
    float v_alpha = cf * d - sf * q;
    float v_beta  = sf * d + cf * q;
    *a =  v_alpha;
    *b = -0.5f * v_alpha + SQRT3_BY_TWO * v_beta;
    *c = -0.5f * v_alpha - SQRT3_BY_TWO * v_beta;
}

void svm(float v_max, float dtc_min, float dtc_max, float over_modulation, float u, float v, float w,
         float *dtc_u, float *dtc_v, float *dtc_w) {
    float v_offset = (fminf(fminf(u, v), w) + fmaxf(fmaxf(u, v), w)) * 0.5f;
    float v_mid    = 0.5f * (dtc_max + dtc_min);
    // *dtc_u = constrain(0.5f * (u - v_offset) * over_modulation / v_max + v_mid, dtc_min, dtc_max);
    // *dtc_v = constrain(0.5f * (v - v_offset) * over_modulation / v_max + v_mid, dtc_min, dtc_max);
    // *dtc_w = constrain(0.5f * (w - v_offset) * over_modulation / v_max + v_mid, dtc_min, dtc_max);
    float scale    = 0.5f * over_modulation / v_max;  // one division, three multiplies
    *dtc_u = constrain((u - v_offset) * scale + v_mid, dtc_min, dtc_max);
    *dtc_v = constrain((v - v_offset) * scale + v_mid, dtc_min, dtc_max);
    *dtc_w = constrain((w - v_offset) * scale + v_mid, dtc_min, dtc_max);
}

void clarke_transform(float ia, float ib, float *i_alpha, float *i_beta) {
    // Clarke transform
    *i_alpha = ia;
    *i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
}

void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float *id, float *iq) {
    // Park transform
    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = i_beta * cos_theta - i_alpha * sin_theta;
}

// Fast combined Clarke + Park Transform
void clarke_park_transform(float ia, float ib, float ic, float sin_theta, float cos_theta, float *id, float *iq) {
    /* 
     * Note: currently using full 3-phase Clarke — zero-sequence error (ia+ib+ic != 0) cancels out
     * Previously, I used the 2-phase Clarke transform, which assumes ia+ib+ic=0.  
     * float i_alpha = ia;
     * float i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
     * This is not always true in practice due to sensor offsets and noise, so the 3-phase version is more robust.
    */
    float i_alpha = (2.0f * ia - ib - ic) * (1.0f/3.0f);
    float i_beta  = (ib - ic) * ONE_BY_SQRT3;

    // Park transform
    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = i_beta * cos_theta - i_alpha * sin_theta;
}
