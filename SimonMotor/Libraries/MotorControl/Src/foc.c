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
#include <math.h>
#include "user_config.h"

void foc_zero_commands(foc_t *hfoc) {
    hfoc->mit_cmd.kd    = 0;
    hfoc->mit_cmd.kp    = 0;
    hfoc->mit_cmd.t_ff  = 0;
    hfoc->mit_cmd.p_des = 0;
    hfoc->mit_cmd.v_des = 0;
}

void foc_sensor_init(foc_t *hfoc) {
	hfoc->angle_sensor.e_zero = E_ZERO_RAD;
    hfoc->angle_sensor.m_zero = M_ZERO_RAD;
	hfoc->angle_sensor.sensor_dir = PHASE_ORDER;
}

void foc_timer_init(foc_t *hfoc, TIM_HandleTypeDef *htim) {
	hfoc->pwm_resolution = __HAL_TIM_GET_AUTORELOAD(htim);
}


void foc_set_pwm_dtc(foc_t *hfoc, float dtc_u, float dtc_v, float dtc_w) {
    __HAL_TIM_SET_COMPARE(&TIM_PWM, PWM_A, (uint32_t)(dtc_u * hfoc->pwm_resolution));
    __HAL_TIM_SET_COMPARE(&TIM_PWM, PWM_B, (uint32_t)(dtc_v * hfoc->pwm_resolution));
    __HAL_TIM_SET_COMPARE(&TIM_PWM, PWM_C, (uint32_t)(dtc_w * hfoc->pwm_resolution));
}

void abc(float sf, float cf, float d, float q, float *a, float *b, float *c) {
    *a =  cf * d - sf * q;
    *b =  (SQRT3_BY_TWO * sf - 0.5f * cf) * d - (-SQRT3_BY_TWO * cf - 0.5f * sf) * q;
    *c = (-SQRT3_BY_TWO * sf - 0.5f * cf) * d - ( SQRT3_BY_TWO * cf - 0.5f * sf) * q;
}

void svm(float v_max, float u, float v, float w,
         float *dtc_u, float *dtc_v, float *dtc_w) {
    float v_offset = (fminf(fminf(u, v), w) + fmaxf(fmaxf(u, v), w)) * 0.5f;
    float v_mid    = 0.5f * (DTC_MAX + DTC_MIN);
    // *dtc_u = CONSTRAIN(0.5f * (u - v_offset) * OVERMODULATION / v_max + v_mid, DTC_MIN, DTC_MAX);
    // *dtc_v = CONSTRAIN(0.5f * (v - v_offset) * OVERMODULATION / v_max + v_mid, DTC_MIN, DTC_MAX);
    // *dtc_w = CONSTRAIN(0.5f * (w - v_offset) * OVERMODULATION / v_max + v_mid, DTC_MIN, DTC_MAX);
    float scale    = 0.5f * OVERMODULATION / v_max;  // one division, three multiplies
    *dtc_u = CONSTRAIN((u - v_offset) * scale + v_mid, DTC_MIN, DTC_MAX);
    *dtc_v = CONSTRAIN((v - v_offset) * scale + v_mid, DTC_MIN, DTC_MAX);
    *dtc_w = CONSTRAIN((w - v_offset) * scale + v_mid, DTC_MIN, DTC_MAX);
}

void foc_set_limit_current(foc_t *hfoc, float i_limit) {
	hfoc->max_current = i_limit;
}


void foc_mit_control_update(foc_t *hfoc){
    hfoc->id_ref = 0.0f;
    float pos_error = hfoc->mit_cmd.p_des - hfoc->angle_sensor.mech_angle_rad;
    float vel_error = hfoc->mit_cmd.v_des - hfoc->angle_sensor.mech_angle_vel;
    float torque_des = hfoc->mit_cmd.kp * pos_error + hfoc->mit_cmd.kd * vel_error + hfoc->mit_cmd.t_ff;
    hfoc->iq_ref = torque_des * (1.0f / (KT*GR));
    // Cap iq_ref for safety
    hfoc->iq_ref = CONSTRAIN(hfoc->iq_ref, -hfoc->max_current, hfoc->max_current);
}

void foc_update_velocity(foc_t *hfoc) {
    // Compute mechanical velocity in rad/s from LUT-corrected angle delta
    angle_sensor_update_velocity(&hfoc->angle_sensor);

    // Restart SPI read if data is ready
    if (MA732_get_val_flag()) {
        MA732_reset_val_flag();
        MA732_start(&hfoc->angle_sensor.ma732);
    }
}

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad) {
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    float va, vb, vc;
    float dtc_u, dtc_v, dtc_w;
    abc(sin_theta, cos_theta, vd_ref, vq_ref, &va, &vb, &vc);
    svm(hfoc->v_bus, va, vb, vc, &dtc_u, &dtc_v, &dtc_w);
    foc_set_pwm_dtc(hfoc, 1.0f-dtc_u, 1.0f-dtc_v, 1.0f-dtc_w); // invert duty cycle because if you want current to flow, you need to drive low (sink current) not high (source current)   
}

/* ── R measurement ───────────────────────────────────────────────────────────
 * Applies a fixed Vd at θ = 0, waits for steady state, then averages Id.
 * Rs = Vd / mean(Id).  Total time: (WARMUP + SAMPLES) / 40000 Hz ≈ 25 ms.
 */
#define R_MEAS_WARMUP   5000u   // cycles for L*dI/dt transient to die out
#define R_MEAS_SAMPLES  500u   // averaging window

static float r_meas_id_buf[R_MEAS_SAMPLES];

void foc_r_meas_update(foc_t *hfoc)
{
    open_loop_voltage_control(hfoc, hfoc->meas_inj_amp, 0.0f, 0.0f);

    CurrentSensor_update(&hfoc->current_sensor);
    float ia = hfoc->current_sensor.ia;
    float ib = hfoc->current_sensor.ib;
    float i_alpha, i_beta, id, iq;
    float sin_th, cos_th;
    pre_calc_sin_cos(0.0f, &sin_th, &cos_th);
    clarke_transform(ia, ib, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, sin_th, cos_th, &id, &iq);

    if (hfoc->meas_inj_n >= R_MEAS_WARMUP) {
        uint32_t idx = hfoc->meas_inj_n - R_MEAS_WARMUP;
        if (idx < R_MEAS_SAMPLES) {
            r_meas_id_buf[idx] = id;
        }
    }

    hfoc->meas_inj_n++;

    if (hfoc->meas_inj_n >= R_MEAS_WARMUP + R_MEAS_SAMPLES) {
        float mean_id = 0.0f;
        for (uint32_t i = 0; i < R_MEAS_SAMPLES; i++) {
            mean_id += r_meas_id_buf[i];
        }
        mean_id /= (float)R_MEAS_SAMPLES;
        if (fabsf(mean_id) > 0.01f) {
            hfoc->Rs = hfoc->meas_inj_amp / mean_id;
        }
        hfoc->meas_inj_n = 0;
        hfoc->meas_done  = 1;
    }
}

/* ── L measurement ───────────────────────────────────────────────────────────
 * Injects a sinusoid on d-axis (Ld) then q-axis (Lq) at L_MEAS_FREQ.
 * The DFT is accumulated incrementally — no large buffers, safe in the ISR.
 *
 *   Z = V_mag / I_mag               (impedance magnitude)
 *   phi = angle(V) - angle(I)       (V leads I by ~90° due to inductance)
 *   L = |Z * sin(phi)| / omega      (inductive reactance / omega)
 *
 * Total measurement time:
 *   2 * (L_MEAS_WARMUP + L_MEAS_SAMPLES) / 40000 Hz ≈ 33 ms
 */
#define L_MEAS_WARMUP      2560u
#define L_MEAS_SAMPLES     4000u      // 10 full cycles at 1000 Hz / 40 kHz
#define L_MEAS_FREQ        1000.0f   // Hz  — ωL >> Rs for most motors at this freq
#define L_MEAS_OMEGA_RADS  (6.28318530718f * L_MEAS_FREQ)           // rad/s
#define L_MEAS_OMEGA_STEP  (L_MEAS_OMEGA_RADS * FOC_TS)             // rad/sample

void foc_l_meas_update(foc_t *hfoc)
{
    /* Pre-alignment: hold DC Vd for L_MEAS_ALIGN cycles to lock the rotor at
     * θ = 0 before AC injection starts.  meas_inj_n counts globally so both
     * the warmup and the DFT collection happen AFTER alignment completes. */
#define L_MEAS_ALIGN  800u   // 20 ms — long enough for rotor to settle at θ=0

    if (hfoc->meas_inj_n < L_MEAS_ALIGN) {
        open_loop_voltage_control(hfoc, hfoc->meas_inj_amp * 0.5f, 0.0f, 0.0f);
        hfoc->meas_inj_n++;
        return;
    }
    /* Offset the cycle counter so warmup/DFT indices start from 0 after align */
    const uint32_t n_ac = hfoc->meas_inj_n - L_MEAS_ALIGN;

    /* Injection angle for this cycle — same for both applying V and DFT reference */
    const float angle = L_MEAS_OMEGA_STEP * (float)n_ac;
    const float sin_a = fast_sin(angle);
    const float cos_a = fast_cos(angle);
    const float v_inj = hfoc->meas_inj_amp * sin_a;

    /* d-axis for Ld phase, q-axis for Lq phase */
    const float vd_ref = (hfoc->l_meas_phase == 0) ? v_inj : 0.0f;
    const float vq_ref = (hfoc->l_meas_phase == 0) ? 0.0f  : v_inj;

    open_loop_voltage_control(hfoc, vd_ref, vq_ref, 0.0f);

    /* Read currents and Park-transform at θ = 0 (same frame as injection) */
    CurrentSensor_update(&hfoc->current_sensor);
    float i_alpha, i_beta, id, iq;
    float sin_th, cos_th;
    pre_calc_sin_cos(0.0f, &sin_th, &cos_th);
    clarke_transform(hfoc->current_sensor.ia, hfoc->current_sensor.ib,
                     &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, sin_th, cos_th, &id, &iq);

    /* Accumulate DFT after warmup */
    if (n_ac >= L_MEAS_WARMUP) {
        const float v = (hfoc->l_meas_phase == 0) ? vd_ref : vq_ref;
        const float i = (hfoc->l_meas_phase == 0) ? id     : iq;
        hfoc->l_meas_Vc += v * cos_a;
        hfoc->l_meas_Vs += v * sin_a;
        hfoc->l_meas_Ic += i * cos_a;
        hfoc->l_meas_Is += i * sin_a;
    }

    hfoc->meas_inj_n++;

    if (n_ac + 1 >= L_MEAS_WARMUP + L_MEAS_SAMPLES) {
        /* Normalise (2/N DFT convention gives peak amplitudes) */
        const float norm  = 2.0f / (float)L_MEAS_SAMPLES;
        const float Vc    = hfoc->l_meas_Vc * norm;
        const float Vs    = hfoc->l_meas_Vs * norm;
        const float Ic    = hfoc->l_meas_Ic * norm;
        const float Is    = hfoc->l_meas_Is * norm;
        const float V_mag = sqrtf(Vc*Vc + Vs*Vs);
        const float I_mag = sqrtf(Ic*Ic + Is*Is);

        if (I_mag > 0.001f) {
            const float phi   = atan2f(Vs, Vc) - atan2f(Is, Ic);
            const float L_est = fabsf(V_mag * sinf(phi) / I_mag) / L_MEAS_OMEGA_RADS;
            if (hfoc->l_meas_phase == 0) hfoc->Ld = L_est;
            else                          hfoc->Lq = L_est;
        }

        /* Reset accumulators; skip re-alignment for the second phase since the
         * rotor is already locked from the Ld injection just before. */
        hfoc->l_meas_Vc = hfoc->l_meas_Vs = 0.0f;
        hfoc->l_meas_Ic = hfoc->l_meas_Is = 0.0f;
        hfoc->meas_inj_n = L_MEAS_ALIGN;   /* jump past alignment phase */

        if (hfoc->l_meas_phase == 1) {
            hfoc->l_meas_phase = 0;   /* proceed to Lq */
        } else {
            hfoc->meas_done = 1;      /* both Ld and Lq done */
        }
    }
}

void foc_current_control_update(foc_t *hfoc) {
	float id_ref = hfoc->id_ref;
	float iq_ref = hfoc->iq_ref;
    const float v_bus = hfoc->v_bus;

    float sin_theta, cos_theta;


    float vd_ref = 0.0f, vq_ref = 0.0f;

    float dtc_u, dtc_v, dtc_w;

    // get currents
    // DRV8302_get_current(&hfoc->drv8302, &ia, &ib);
    CurrentSensor_update(&hfoc->current_sensor);


    // Hard limit references
    id_ref = CONSTRAIN(id_ref, -hfoc->max_current, hfoc->max_current);
    iq_ref = CONSTRAIN(iq_ref, -hfoc->max_current, hfoc->max_current);

    // pre calculate sin & cos
    pre_calc_sin_cos(hfoc->angle_sensor.e_rad, &sin_theta, &cos_theta);
    // pre_calc_sin_cos(0.0f, &sin_theta, &cos_theta);

    float id, iq;
    clarke_park_transform(hfoc->current_sensor.ia, hfoc->current_sensor.ib,
                          sin_theta, cos_theta, &id, &iq);
    

    // set dynamic max output vd and vq
    hfoc->id_ctrl.out_max = hfoc->id_ctrl.out_max_dynamic * v_bus;
    hfoc->iq_ctrl.out_max = hfoc->iq_ctrl.out_max_dynamic * v_bus;

    vd_ref = pi_control(&hfoc->id_ctrl, id_ref - id);
    vq_ref = pi_control(&hfoc->iq_ctrl, iq_ref - iq);

    float va, vb, vc;
    abc(sin_theta, cos_theta, vd_ref, vq_ref, &va, &vb, &vc);
    svm(v_bus, va, vb, vc, &dtc_u, &dtc_v, &dtc_w);
    foc_set_pwm_dtc(hfoc, 1.0f-dtc_u, 1.0f-dtc_v, 1.0f-dtc_w); // invert duty cycle because if you want current to flow, you need to drive low (sink current) not high (source current)
    

    // copy to struct for debug
    hfoc->ia = hfoc->current_sensor.ia;
    hfoc->ib = hfoc->current_sensor.ib;
    hfoc->ic = -hfoc->current_sensor.ia - hfoc->current_sensor.ib;

    hfoc->id = id;
    hfoc->iq = iq;

}
