/*
 * foc.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#include "foc.h"
#include "angle_sensor.h"
#include "main.h"
#include "hw_config.h"
#include "tim.h"

void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv) {
	if (hfoc == NULL || pole_pairs == 0 || kv <= 0) {
		return;
	}

	hfoc->angle_sensor.pole_pairs = pole_pairs;
	hfoc->kv = kv;
}

void foc_sensor_init(foc_t *hfoc, float e_zero_rad, dir_mode_t sensor_dir) {
	if (hfoc == NULL) return;

	hfoc->angle_sensor.e_zero = e_zero_rad;
	hfoc->angle_sensor.sensor_dir = sensor_dir;
}

void foc_timer_init(foc_t *hfoc, TIM_HandleTypeDef *htim) {
	if (hfoc == NULL || htim == NULL) {
		return;
	}

	hfoc->timer = htim;
	hfoc->pwm_resolution = __HAL_TIM_GET_AUTORELOAD(htim);
}

void foc_set_pwm(foc_t *hfoc, uint32_t da, uint32_t db, uint32_t dc) {
    __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, da);
    __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, db);
    __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, dc);
}

void foc_set_pwm_dtc(foc_t *hfoc, float dtc_u, float dtc_v, float dtc_w) {
    uint32_t res = hfoc->pwm_resolution;
    __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, (uint32_t)(dtc_u * res));
    __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, (uint32_t)(dtc_v * res));
    __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, (uint32_t)(dtc_w * res));
}

void foc_set_limit_current(foc_t *hfoc, float i_limit) {
	if (hfoc == NULL) return;

	hfoc->max_current = i_limit;
}

void foc_speed_control_update(foc_t *hfoc, float vel_reference) {
	if (hfoc == NULL || (hfoc->control_mode != SPEED_CONTROL_MODE && hfoc->control_mode != POSITION_CONTROL_MODE)) {
		hfoc->speed_ctrl.integral = 0.0f;
		hfoc->speed_ctrl.last_error = 0.0f;
		return;
	}

    hfoc->id_ref = 0.0f;
    hfoc->iq_ref = pid_control(&hfoc->speed_ctrl, vel_reference - hfoc->angle_sensor.actual_vel);
}

void foc_mit_control_update(foc_t *hfoc){
    if (hfoc == NULL) return;
    hfoc->id_ref = 0.0f;
    float pos_error = hfoc->mit_cmd.des_pos - hfoc->angle_sensor.m_angle_rad;
    // Wrap to [-π, π] so the controller always takes the shortest path
    while (pos_error >  PI) pos_error -= TWO_PI;
    while (pos_error < -PI) pos_error += TWO_PI;
    float vel_error = hfoc->mit_cmd.des_vel - hfoc->angle_sensor.actual_vel;
    hfoc->iq_ref = hfoc->mit_cmd.kp * pos_error + hfoc->mit_cmd.kd * vel_error;
    // Cap iq_ref for safety
    hfoc->iq_ref = CONSTRAIN(hfoc->iq_ref, -10.0f, 10.0f);
}

void foc_update_position_velocity(foc_t *hfoc, float Ts) {
    if (hfoc == NULL || Ts <= 0.0f) {
        return;
    }

    // Latch the latest normalised electric angle for the current controller
    hfoc->angle_sensor.e_rad = hfoc->angle_sensor.e_angle_rad_comp;

    // Compute mechanical velocity in rad/s from LUT-corrected angle delta
    angle_sensor_update_velocity(&hfoc->angle_sensor, Ts);

    // Restart SPI read if data is ready
    if (MA732_get_val_flag()) {
        MA732_reset_val_flag();
        MA732_start(&hfoc->angle_sensor.ma732);
    }
}

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad) {
    uint32_t da, db, dc;
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, hfoc->v_bus, hfoc->pwm_resolution, &da, &db, &dc);
    foc_set_pwm(hfoc, da, db, dc);
}

void foc_current_control_update(foc_t *hfoc, float Ts) {
	if (hfoc == NULL || Ts <= 0.0f) {
		hfoc->id_ctrl.integral = 0.0f;
		hfoc->id_ctrl.last_error = 0.0f;
		hfoc->iq_ctrl.integral = 0.0f;
		hfoc->iq_ctrl.last_error = 0.0f;
		return;
	}

	float id_ref = hfoc->id_ref;
	float iq_ref = hfoc->iq_ref;
    const float v_bus = hfoc->v_bus;

    float ia, ib;
    float sin_theta, cos_theta;
    float i_alpha, i_beta;
    float id, iq;

    float id_error = 0.0f, iq_error = 0.0f;
    float vd_ref = 0.0f, vq_ref = 0.0f;

    float dtc_u, dtc_v, dtc_w;

    // get currents
    // DRV8302_get_current(&hfoc->drv8302, &ia, &ib);
    CurrentSensor_update(&hfoc->current_sensor);
    ia = hfoc->current_sensor.ia;
    ib = hfoc->current_sensor.ib;

    // Hard limit references
    id_ref = CONSTRAIN(id_ref, -hfoc->max_current, hfoc->max_current);
    iq_ref = CONSTRAIN(iq_ref, -hfoc->max_current, hfoc->max_current);

    // pre calculate sin & cos
    pre_calc_sin_cos(hfoc->angle_sensor.e_rad, &sin_theta, &cos_theta);
    // pre_calc_sin_cos(0.0f, &sin_theta, &cos_theta);

    clarke_transform(ia, ib, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, sin_theta, cos_theta, &id, &iq);
    
    
    id_error = id_ref - id;
    iq_error = iq_ref - iq;

    // set dynamic max output vd and vq
    hfoc->id_ctrl.out_max = hfoc->id_ctrl.out_max_dynamic * v_bus;
    hfoc->iq_ctrl.out_max = hfoc->iq_ctrl.out_max_dynamic * v_bus;

    vd_ref = pi_control(&hfoc->id_ctrl, id_error);
    vq_ref = pi_control(&hfoc->iq_ctrl, iq_error);

    float va, vb, vc;
    abc(hfoc->angle_sensor.e_rad, vd_ref, vq_ref, &va, &vb, &vc);
    svm(v_bus, va, vb, vc, &dtc_u, &dtc_v, &dtc_w);
    foc_set_pwm_dtc(hfoc, dtc_u, dtc_v, dtc_w);
    

    // copy to struct for debug
    hfoc->ia = ia;
    hfoc->ib = ib;
    hfoc->ic = -ia - ib;
    hfoc->i_alpha = i_alpha;
    hfoc->i_beta = i_beta;
    hfoc->id = id;
    hfoc->iq = iq;

    hfoc->va = dtc_u * v_bus;
    hfoc->vb = dtc_v * v_bus;
    hfoc->vc = dtc_w * v_bus;
    hfoc->vd = vd_ref;
    hfoc->vq = vq_ref;
}
