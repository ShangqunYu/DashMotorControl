
#include "math_ops.h"
#include "lookup.h"


float fast_fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fast_fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }

void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrtf(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }
    
void limit(float *x, float min, float max){
    *x = fast_fmaxf(fast_fminf(*x, max), min);
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

float sin_lut(float theta){
	theta = fmodf(theta, TWO_PI_F);
	theta = theta<0 ? theta + TWO_PI_F : theta;

	return sin_tab[(int) (LUT_MULT*theta)];
}

float cos_lut(float theta){
	return sin_lut(PI_OVER_2_F - theta);
}
