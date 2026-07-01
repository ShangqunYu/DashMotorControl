/*
 * foc.h
 *
 *  Created on: Jul 01, 2026
 *      Author: Simon
 */

#ifndef FOC_INC_H_
#define FOC_INC_H_




void abc(float sf, float cf, float d, float q, float *a, float *b, float *c);
void svm(float v_max, float dtc_min, float dtc_max, float over_modulation, float u, float v, float w,
         float *dtc_u, float *dtc_v, float *dtc_w);
void clarke_transform(float ia, float ib, float *i_alpha, float *i_beta);
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float *id, float *iq);
void clarke_park_transform(float ia, float ib, float ic, float sin_theta, float cos_theta, float *id, float *iq);


#endif /* FOC_INC_H_ */
