/*
 * foc.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_H_
#define FOC_INC_FOC_H_

#include <stdint.h>
#include "pid_utils.h"
#include "lpf.h"
#include "math_utils.h"
#include "hw_config.h"
#include "angle_sensor.h"      /* AngleSensor_t, dir_mode_t */
#include "analog_sensor.h"     /* CurrentSensor */


typedef enum {
	MENU_MODE,           // idle / serial menu
	CALIBRATION_MODE,
	MIT_MODE,
	ENCODER_MODE,    // motor coast; prints raw vs LUT-compensated angle
	SET_ZERO_MODE,   // capture current position as mechanical zero
	SETUP_MODE,      // serial parameter configuration
	R_MEAS_MODE,     // DC winding resistance measurement
	L_MEAS_MODE,     // AC winding inductance measurement (Ld then Lq)
} motor_state;

typedef struct {
    union{
    	float commands[5];									// Making this easier to pass around without including foc.h everywhere
    	struct{
    		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
    	};
    };	
} MIT_CMD;

typedef struct {
	AngleSensor_t angle_sensor;  /* MA732 + all angle/velocity state */
    CurrentSensor current_sensor;

	float max_current;
	float ia, ib, ic;
	float id, iq;
	float id_ref, iq_ref;

	float v_bus;
	float motor_temp;

	PID_Controller_t id_ctrl, iq_ctrl;

    uint32_t pwm_resolution;

	MIT_CMD          mit_cmd;
	volatile MIT_CMD mit_buf;
	volatile uint8_t new_cmd;

	/* ── R/L measurement ── */
	float Rs, Ld, Lq;
	float    meas_inj_amp;  // injection voltage (V)
	uint32_t meas_inj_n;   // sample / cycle counter (written by ISR)
	uint8_t  meas_done;    // set 1 by ISR when measurement complete

	/* running DFT accumulators — reused for Ld then Lq */
	float    l_meas_Vc, l_meas_Vs;  // voltage cosine / sine component
	float    l_meas_Ic, l_meas_Is;  // current cosine / sine component
	uint8_t  l_meas_phase;           // 0 = collecting Ld, 1 = collecting Lq
} foc_t;

void foc_zero_commands(foc_t *hfoc);
void foc_set_limit_current(foc_t *hfoc, float i_limit);
void foc_r_meas_update(foc_t *hfoc);
void foc_l_meas_update(foc_t *hfoc);
void foc_timer_init(foc_t *hfoc, TIM_HandleTypeDef *htim);
void foc_set_pwm_dtc(foc_t *hfoc, float dtc_u, float dtc_v, float dtc_w);
void abc(float sf, float cf, float d, float q, float *a, float *b, float *c);
void svm(float v_max, float u, float v, float w,
         float *dtc_u, float *dtc_v, float *dtc_w);
void clarke_transform(float ia, float ib, float *i_alpha, float *i_beta);
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float *id, float *iq);
void clarke_park_transform(float ia, float ib, float sin_theta, float cos_theta, float *id, float *iq);
void foc_mit_control_update(foc_t *hfoc);
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);
void foc_current_control_update(foc_t *hfoc);

#endif /* FOC_INC_FOC_H_ */
