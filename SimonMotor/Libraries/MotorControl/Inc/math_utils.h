/*
 * math_utils.h
 *
 *  Created on: Jun 28, 2026
 *      Author: Simon
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_


#include <stdint.h>

#define LUT_SIZE (1024*4)
#define LUT_STEP (TWO_PI / (float)LUT_SIZE)

#define TWO_BY_SQRT3 1.15470053838f
#define ONE_BY_SQRT3 0.57735026919f
#define SQRT3_BY_TWO 0.86602540378f
#define TWO_PI 6.2831853f
#define PI 3.1415926f

extern float sin_lut_new[LUT_SIZE];
extern float cos_lut_new[LUT_SIZE];
float constrain(float val, float min, float max);
void init_trig_lut(void);
void norm_angle_rad(float *theta);
void fast_sincos(float theta, float *s, float *c);
float fast_sin(float theta);
float fast_cos(float theta);
float fast_atan2(float y, float x);
void pre_calc_sin_cos(float theta, float *sin_theta, float *cos_theta);

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
#endif 
