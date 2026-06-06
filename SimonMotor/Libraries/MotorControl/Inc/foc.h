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
#include "FOC_math.h"
#include "hw_config.h"
#include "angle_sensor.h"      /* AngleSensor_t, dir_mode_t */
#include "analog_sensor.h"     /* CurrentSensor */


typedef enum {
	MENU_MODE,           // idle / serial menu
	TORQUE_CONTROL_MODE,
	SPEED_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	CALIBRATION_MODE,
	MIT_MODE,
	ENCODER_MODE,    // motor coast; prints raw vs LUT-compensated angle
	SET_ZERO_MODE,   // capture current position as mechanical zero
	SETUP_MODE,      // serial parameter configuration
} motor_state;

typedef enum {
  RS, LD, LQ
}inject_taregt_t;

// state machine for polarity detection
typedef enum {
	P_DET_START,
	P_DET_POSITIVE,
	P_DET_WAITING_POSITIVE,
	P_DET_NEGATIVE,
	P_DET_WAITING_NEGATIVE,
	P_DET_STOP
}p_det_state_t;

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
	float kv;
	float Rs;
	float Ld;
	float Lq;
	float max_current;
	float flux_linkage;

	float vd, vq;
	float id, iq;
	float id_filtered, iq_filtered;
	float v_alpha, v_beta;
	float i_alpha, i_beta;
	float va, vb, vc;
	float ia, ib, ic;
	float v_bus;
	float motor_temp;
	float i_bus;

	float I_ctrl_bandwidth;
	float id_ref, iq_ref;
	float vel_ref;   // velocity reference in rad/s

    uint8_t loop_count;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;

	float gear_ratio;

	//polarity detection
	p_det_state_t pd_state;
	float pd_v_pulse;
	float pd_i_p;
	float pd_i_n;
	uint16_t pd_time;
	uint16_t pd_count;

    uint32_t pwm_resolution;
    TIM_HandleTypeDef *timer;

	//debug
	int sample_index;
	_Bool collect_sample_flag;

	MIT_CMD          mit_cmd;
	volatile MIT_CMD mit_buf;
	volatile uint8_t new_cmd;
} foc_t;

void foc_zero_commands(foc_t *hfoc);
void foc_set_limit_current(foc_t *hfoc, float i_limit);
void foc_sensor_init(foc_t *hfoc, float e_zero_rad, dir_mode_t sensor_dir);
void foc_timer_init(foc_t *hfoc, TIM_HandleTypeDef *htim);
void foc_set_pwm(foc_t *hfoc, uint32_t da, uint32_t db, uint32_t dc);
void foc_set_pwm_dtc(foc_t *hfoc, float dtc_u, float dtc_v, float dtc_w);
void abc(float theta, float d, float q, float *a, float *b, float *c);
void svm(float v_max, float u, float v, float w,
         float *dtc_u, float *dtc_v, float *dtc_w);
void foc_speed_control_update(foc_t *hfoc, float vel_reference);
void foc_update_velocity(foc_t *hfoc, float Ts);
void foc_mit_control_update(foc_t *hfoc);
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);
void foc_current_control_update(foc_t *hfoc, float Ts);

#endif /* FOC_INC_FOC_H_ */
