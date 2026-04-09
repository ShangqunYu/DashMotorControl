/*
 * foc.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#include "foc.h"
#include "main.h"

void CurrentSensor_init(CurrentSensor *sensor,
                        volatile uint32_t *adc_ia,
                        volatile uint32_t *adc_ib,
                        float current_scale,
                        int adc_a_offset,
                        int adc_b_offset) {
    sensor->adc_ia = adc_ia;
    sensor->adc_ib = adc_ib;
    sensor->current_scale = current_scale;
    sensor->adc_a_offset = adc_a_offset;
    sensor->adc_b_offset = adc_b_offset;
    sensor->ia = 0.0f;
    sensor->ib = 0.0f;
    sensor->ic = 0.0f;
    sensor->ia_filtered = 0.0f;
    sensor->ib_filtered = 0.0f;
    sensor->ic_filtered = 0.0f;
    sensor->offset_calibrating = 0U;
    sensor->adc_a_offset_sum = 0U;
    sensor->adc_b_offset_sum = 0U;
    sensor->adc_offset_sample_count = 0U;
}

void CurrentSensor_update(CurrentSensor *sensor) {
    sensor->ia = -(sensor->adc_a_offset -(float)(*sensor->adc_ia)) * sensor->current_scale;
    sensor->ib = -(sensor->adc_b_offset -(float)(*sensor->adc_ib)) * sensor->current_scale;
    sensor->ic = -sensor->ia - sensor->ib;
    // low-pass filter
    sensor->ia_filtered = (1.0f - CURRENT_FILTER_ALPHA) * sensor->ia_filtered + CURRENT_FILTER_ALPHA * sensor->ia;
    sensor->ib_filtered = (1.0f - CURRENT_FILTER_ALPHA) * sensor->ib_filtered + CURRENT_FILTER_ALPHA * sensor->ib;
    sensor->ic_filtered = -sensor->ia_filtered - sensor->ib_filtered;
}

void CurrentSensor_begin_offset_calibration(CurrentSensor *sensor) {
    sensor->offset_calibrating = 1U;
    sensor->adc_a_offset_sum = 0U;
    sensor->adc_b_offset_sum = 0U;
    sensor->adc_offset_sample_count = 0U;
}

void CurrentSensor_sample_offset(CurrentSensor *sensor) {
    if (sensor->offset_calibrating == 0U) {
        return;
    }

    sensor->adc_a_offset_sum += *sensor->adc_ia;
    sensor->adc_b_offset_sum += *sensor->adc_ib;
    sensor->adc_offset_sample_count++;
}

void CurrentSensor_end_offset_calibration(CurrentSensor *sensor) {
    sensor->offset_calibrating = 0U;

    if (sensor->adc_offset_sample_count > 0U) {
        sensor->adc_a_offset = (int)(sensor->adc_a_offset_sum / sensor->adc_offset_sample_count);
        sensor->adc_b_offset = (int)(sensor->adc_b_offset_sum / sensor->adc_offset_sample_count);
    }
}

void CurrentSensor_calibrate(CurrentSensor *sensor, uint32_t duration_ms) {
    uint32_t start_tick = HAL_GetTick();

    CurrentSensor_begin_offset_calibration(sensor);

    while ((HAL_GetTick() - start_tick) < duration_ms) {
        HAL_Delay(1);
    }

    CurrentSensor_end_offset_calibration(sensor);
}

void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv) {
	if (hfoc == NULL || pole_pairs == 0 || kv <= 0) {
		return;
	}

	hfoc->pole_pairs = pole_pairs;
	hfoc->kv = kv;
}

void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir) {
	if (hfoc == NULL) return;

	hfoc->m_angle_offset = m_rad_offset;
	hfoc->sensor_dir = sensor_dir;
}

void foc_timer_init(foc_t *hfoc, TIM_HandleTypeDef *htim) {
	if (hfoc == NULL || htim == NULL) {
		return;
	}

	hfoc->timer = htim;
	hfoc->pwm_resolution = __HAL_TIM_GET_AUTORELOAD(htim);
}

void foc_set_pwm(foc_t *hfoc, uint32_t da, uint32_t db, uint32_t dc) {
    hfoc->timer->Instance->CCR1 = da;
    hfoc->timer->Instance->CCR2 = db;
    hfoc->timer->Instance->CCR3 = dc;
}

void foc_set_limit_current(foc_t *hfoc, float i_limit) {
	if (hfoc == NULL) return;

	hfoc->max_current = i_limit;
}

void foc_sensored_calc_electric_angle(foc_t *hfoc) {


    float angle_deg = MA732_get_degree(&hfoc->ma732);

    // Normalize mechanical angle
    hfoc->m_angle_rad = DEG_TO_RAD(angle_deg) - hfoc->m_angle_offset;
    norm_angle_rad(&hfoc->m_angle_rad);

    // Calculate raw electric angle
    float e_rad = hfoc->m_angle_rad * hfoc->pole_pairs;
    
    // Handle sensor direction
    if (hfoc->sensor_dir == REVERSE_DIR) {
        e_rad = TWO_PI - e_rad;
    }

    hfoc->e_angle_rad = e_rad;

    // Calculate LUT index with wrap-around
    // float lut_idx_f = (hfoc->m_angle_rad / TWO_PI) * ERROR_LUT_SIZE;
    // lut_idx_f = fmodf(lut_idx_f, ERROR_LUT_SIZE);
    // if (lut_idx_f < 0) {
    //     lut_idx_f += ERROR_LUT_SIZE;
    // }

    // Get neighboring indices with wrap-around
    // int idx0 = (int)lut_idx_f % ERROR_LUT_SIZE;
    // int idx1 = (idx0 + 1) % ERROR_LUT_SIZE;
    // float frac = lut_idx_f - (float)idx0;

    // Linear interpolation
    // float encoder_error = m_config.encd_error_comp[idx0] * (1.0f - frac) + m_config.encd_error_comp[idx1] * frac;
    // e_rad += encoder_error;
    
    // Normalize final electric angle
    norm_angle_rad(&e_rad);

    hfoc->e_angle_rad_comp = e_rad;
    
    MA732_reset_val_flag();
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
	if (hfoc == NULL || Ts <= 0.0f || hfoc->control_mode == AUDIO_MODE) {
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

    uint32_t da, db, dc;
    uint32_t pwm_res = hfoc->pwm_resolution;
    const float pwm_to_v = v_bus / (float)pwm_res;

    // get currents
    // DRV8302_get_current(&hfoc->drv8302, &ia, &ib);
    CurrentSensor_update(&hfoc->current_sensor);
    ia = hfoc->current_sensor.ia;
    ib = hfoc->current_sensor.ib;

    // Hard limit references
    id_ref = CONSTRAIN(id_ref, -hfoc->max_current, hfoc->max_current);
    iq_ref = CONSTRAIN(iq_ref, -hfoc->max_current, hfoc->max_current);

    // pre calculate sin & cos
    // pre_calc_sin_cos(hfoc->e_rad, &sin_theta, &cos_theta);
    pre_calc_sin_cos(0.0f, &sin_theta, &cos_theta);

    clarke_transform(ia, ib, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, sin_theta, cos_theta, &id, &iq);
    
    
    id_error = id_ref - id;
    iq_error = iq_ref - iq;
    hfoc->e_rad = hfoc->e_angle_rad_comp;
    hfoc->actual_rpm = MA732_get_rpm(&hfoc->ma732, Ts);
    if (hfoc->sensor_dir == REVERSE_DIR) {
        hfoc->actual_rpm = -hfoc->actual_rpm;
    }

    if (MA732_get_val_flag()) {
        MA732_reset_val_flag();
        MA732_start(&hfoc->ma732);
    }


    // set dynamic max output vd and vq
    hfoc->id_ctrl.out_max = hfoc->id_ctrl.out_max_dynamic * v_bus;
    hfoc->iq_ctrl.out_max = hfoc->iq_ctrl.out_max_dynamic * v_bus;

    vd_ref = pi_control(&hfoc->id_ctrl, id_error);
    vq_ref = pi_control(&hfoc->iq_ctrl, iq_error);

    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, v_bus, pwm_res, &da, &db, &dc);
    foc_set_pwm(hfoc, dc, db, da);
    // foc_set_pwm(hfoc, 500, 0, 0);
    

    // copy to struct for debug
    hfoc->ia = ia;
    hfoc->ib = ib;
    hfoc->ic = -ia - ib;
    hfoc->i_alpha = i_alpha;
    hfoc->i_beta = i_beta;
    hfoc->id = id;
    hfoc->iq = iq;

    hfoc->va = da * pwm_to_v;
    hfoc->vb = db * pwm_to_v;
    hfoc->vc = dc * pwm_to_v;
    hfoc->vd = vd_ref;
    hfoc->vq = vq_ref;
}
