/*
 * PMSM_motor.h
 *
 *  Created on: Jul 01, 2026
 *      Author: Simon
 */

#ifndef PMSM_MOTOR_INC_H_
#define PMSM_MOTOR_INC_H_

#include <stdint.h>
#include "pid_utils.h"
#include "math_utils.h"
#include "hw_config.h"
#include "angle_sensor.h"      /* AngleSensor_t, dir_mode_t */
#include "analog_sensor.h"     /* CurrentSensor */
#include "drv8353.h"

typedef enum {
	MENU_MODE        = 0,
	CALIBRATION_MODE = 1,
	MIT_MODE         = 2,
	ENCODER_MODE     = 3,
	SET_ZERO_MODE    = 4,
	R_MEAS_MODE      = 5,
	L_MEAS_MODE      = 6,
	NO_PENDING_MODE  = 0xFF,
} motor_state;

typedef struct {
    union{
    	float commands[5];									// Making this easier to pass around without including PMSM_motor.h everywhere
    	struct{
    		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
    	};
    };	
} MIT_CMD;


typedef struct {
	MIT_CMD curr_cmd;
	volatile MIT_CMD cmd_buf;
	volatile uint8_t new_cmd;
	volatile uint8_t pending_fsm_cmd;
} MOTOR_CMD;

typedef struct {
    motor_state curr_state;
    motor_state next_state;
} FSMStruct;

typedef struct {
	AngleSensor_t angle_sensor;  /* MA732 + all angle/velocity state */
    CurrentSensor current_sensor;
	MOTOR_CMD cmd;
	FSMStruct fsm;
	GateDriver gateDriver;

	float max_current;
	float id, iq;
	float id_ref, iq_ref;

	float v_bus;
	float motor_temp;

	PID_Controller_t id_ctrl, iq_ctrl;

    uint32_t pwm_resolution;



	/* ── R/L measurement ── */
	float Rs, Ld, Lq;
	float    meas_inj_amp;  // injection voltage (V)
	uint32_t meas_inj_n;   // sample / cycle counter (written by ISR)
	uint8_t  meas_done;    // set 1 by ISR when measurement complete

	/* running DFT accumulators — reused for Ld then Lq */
	float    l_meas_Vc, l_meas_Vs;  // voltage cosine / sine component
	float    l_meas_Ic, l_meas_Is;  // current cosine / sine component
	uint8_t  l_meas_phase;           // 0 = collecting Ld, 1 = collecting Lq
} PMSM_motor;

void zero_commands(PMSM_motor *motor);
void set_limit_current(PMSM_motor *motor, float i_limit);
void timer_init(PMSM_motor *motor, TIM_HandleTypeDef *htim);
void set_pwm_dtc(PMSM_motor *motor, float dtc_u, float dtc_v, float dtc_w);
void mit_control_update(PMSM_motor *motor);
void open_loop_voltage_control(PMSM_motor *motor, float vd_ref, float vq_ref, float angle_rad);
void current_control_update(PMSM_motor *motor);

#endif /* PMSM_MOTOR_INC_H_ */
