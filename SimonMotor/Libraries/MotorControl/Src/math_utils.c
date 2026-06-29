/*
 * math_utils.c
 *
 *  Created on: Jun 28, 2026
 *      Author: Simon
 */

#include "math_utils.h"
#include <math.h>

float sin_lut_new[LUT_SIZE];
float cos_lut_new[LUT_SIZE];

void init_trig_lut(void) {
    for (int i = 0; i < LUT_SIZE; ++i) {
        float angle = i * LUT_STEP;
        sin_lut_new[i] = sinf(angle);
        cos_lut_new[i] = cosf(angle);
    }
}

void norm_angle_rad(float *theta) {
    if (!isfinite(*theta)) { *theta = 0.0f; return; }
    *theta = fmodf(*theta, TWO_PI);
    if (*theta < 0.0f) *theta += TWO_PI;
}

void fast_sincos(float theta, float *s, float *c) {
    norm_angle_rad(&theta);
    float index_f = theta / LUT_STEP;
    int   index   = (int)index_f;
    float frac    = index_f - (float)index;
    int   next    = index + 1;
    if (next >= LUT_SIZE) next = 0;
    float f1 = 1.0f - frac;
    *s = sin_lut_new[index] * f1 + sin_lut_new[next] * frac;
    *c = cos_lut_new[index] * f1 + cos_lut_new[next] * frac;
}

float fast_sin(float theta) { float s, c; fast_sincos(theta, &s, &c); return s; }
float fast_cos(float theta) { float s, c; fast_sincos(theta, &s, &c); return c; }

void pre_calc_sin_cos(float theta, float *sin_theta, float *cos_theta) {
    fast_sincos(theta, sin_theta, cos_theta);
}

float fast_atan2(float y, float x) {
    if (x == 0.0f) {
        return (y > 0.0f) ? PI/2.0f : -PI/2.0f;
    }
    
    float abs_x = fabsf(x);
    float abs_y = fabsf(y);
    float a = fminf(abs_x, abs_y) / fmaxf(abs_x, abs_y);
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    
    if (abs_y > abs_x) {
        r = PI/2.0f - r;
    }
    
    if (x < 0.0f) {
        r = PI - r;
    }
    
    if (y < 0.0f) {
        r = -r;
    }
    
    return r;
}


float constrain(float val, float min, float max) {
    return ((val) <= (min) ? (min) : ((val) >= (max) ? (max) : (val)));
}
    

int float_to_uint(float x, float x_min, float x_max, int bits){
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (int)((x - x_min) * (float)((1<<bits)-1) / (x_max - x_min));
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    int max_int = (1<<bits) - 1;
    if (x_int < 0)        x_int = 0;
    if (x_int > max_int)  x_int = max_int;
    return (float)x_int * (x_max - x_min) / (float)max_int + x_min;
}