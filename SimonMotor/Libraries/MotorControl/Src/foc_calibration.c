/*
 * foc_calibration.c
 *
 * Encoder offset + nonlinearity LUT calibration (Ben Katz method).
 *
 * Phase 1 – offset:
 *   Lock rotor at electrical angle 0, average CAL_ITERATION raw angle
 *   readings to find m_angle_offset.
 *
 * Phase 2 – LUT sweep:
 *   Rotate CW then CCW at constant W_CAL (electrical rad/s), collecting
 *   LUT_SAMPLES_PER_PPAIR samples per pole-pair.  CW and CCW are averaged
 *   to cancel friction bias.
 *
 * Phase 3 – post-processing (call foc_cal_lut_postprocess from main loop):
 *   Resample raw errors onto the ERROR_LUT_SIZE grid with a moving-average
 *   window of one pole-pair width, which cancels cogging-torque ripple.
 */

#include "foc_calibration.h"
#include "foc.h"
#include "main.h"

void foc_cal_encoder_misalignment_start(foc_t *hfoc, CalStruct *hcal) {
    if (hfoc == NULL || hcal == NULL) return;

    hcal->cal_state          = CAL_STATE_SETTLING;
    hcal->cal_start_time     = HAL_GetTick();
    hcal->cal_sample_count   = 0;
    hcal->cal_rad_offset_sum = 0.0f;

    open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
}

void foc_cal_encoder_misalignment_update(foc_t *hfoc, CalStruct *hcal, float Ts) {
    if (hfoc == NULL || hcal == NULL) return;

    uint32_t elapsed_time = HAL_GetTick() - hcal->cal_start_time;

    // ── Phase 1: offset calibration ────────────────────────────────────────

    // Settling: wait 1 s for rotor to lock at electrical angle 0
    if (hcal->cal_state == CAL_STATE_SETTLING) {
        if (elapsed_time >= 1000) {
            hcal->cal_state          = CAL_STATE_SAMPLING;
            hcal->cal_start_time     = HAL_GetTick();
            hcal->cal_sample_count   = 0;
            hcal->cal_rad_offset_sum = 0.0f;
        }
        return;
    }

    // Sampling: collect CAL_ITERATION samples and average for the offset
    if (hcal->cal_state == CAL_STATE_SAMPLING) {
        if (hcal->cal_sample_count < CAL_ITERATION) {
            hcal->cal_rad_offset_sum += DEG_TO_RAD(hfoc->angle_sensor.ma732.angle_filtered);
            hcal->cal_sample_count++;
        } else {
            hcal->cal_state = CAL_STATE_COMPLETE;
        }
        return;
    }

    // Offset done: store result, lock at e=0, start LUT settle timer
    if (hcal->cal_state == CAL_STATE_COMPLETE) {
        hfoc->angle_sensor.m_angle_offset = hcal->cal_rad_offset_sum / (float)hcal->cal_sample_count;
        hcal->lut_cal_idx                 = 0;
        hfoc->angle_sensor.lut_ready      = 0;  // clear old LUT until sweep completes
        hcal->lut_theta_ref               = 0.0f;
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
        hcal->cal_start_time              = HAL_GetTick();
        hcal->cal_state                   = CAL_STATE_LUT_SETTLING;
        return;
    }

    // ── Phase 2: nonlinearity LUT (Ben Katz method) ────────────────────────

    // LUT_SETTLING: hold at e=0 for 1 s before starting the sweep
    if (hcal->cal_state == CAL_STATE_LUT_SETTLING) {
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->lut_theta_ref         = 0.0f;
            hcal->lut_next_sample_e_rad = 0.0f;
            hcal->lut_cal_idx           = 0;
            hcal->cal_state             = CAL_STATE_LUT_CW;
        }
        return;
    }

    // LUT_CW: advance voltage angle CW; record a raw sample every
    //         2π / LUT_SAMPLES_PER_PPAIR electrical radians
    if (hcal->cal_state == CAL_STATE_LUT_CW) {
        hcal->lut_theta_ref += W_CAL * Ts;
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, hcal->lut_theta_ref);

        if (hcal->lut_theta_ref >= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->lut_cal_idx;
            float ideal  = (float)idx * TWO_PI / (float)LUT_NUM_SAMPLES;
            float err    = hfoc->angle_sensor.m_angle_rad - ideal;
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;
            hcal->lut_raw[idx] = err;

            hcal->lut_next_sample_e_rad += TWO_PI / (float)LUT_SAMPLES_PER_PPAIR;

            if (++hcal->lut_cal_idx >= (uint16_t)LUT_NUM_SAMPLES) {
                hcal->lut_cal_idx           = (uint16_t)(LUT_NUM_SAMPLES - 1);
                hcal->lut_next_sample_e_rad = hcal->lut_theta_ref;
                hcal->cal_state             = CAL_STATE_LUT_CCW;
            }
        }
        return;
    }

    // LUT_CCW: reverse rotation; average each sample with its CW counterpart
    if (hcal->cal_state == CAL_STATE_LUT_CCW) {
        hcal->lut_theta_ref -= W_CAL * Ts;
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, hcal->lut_theta_ref);

        if (hcal->lut_theta_ref <= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->lut_cal_idx;
            float ideal  = (float)idx * TWO_PI / (float)LUT_NUM_SAMPLES;
            float err    = hfoc->angle_sensor.m_angle_rad - ideal;
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;
            hcal->lut_raw[idx] = (hcal->lut_raw[idx] + err) * 0.5f;

            hcal->lut_next_sample_e_rad -= TWO_PI / (float)LUT_SAMPLES_PER_PPAIR;

            if (hcal->lut_cal_idx == 0) {
                open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
                hcal->cal_state = CAL_STATE_LUT_POSTPROC_PENDING;
            } else {
                hcal->lut_cal_idx--;
            }
        }
        return;
    }

    // POSTPROC_PENDING: keep motor stopped; main loop calls foc_cal_lut_postprocess()
    if (hcal->cal_state == CAL_STATE_LUT_POSTPROC_PENDING) {
        open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
        return;
    }
}

// ── Post-processing (call from main loop, NOT from an ISR) ─────────────────
void foc_cal_lut_postprocess(foc_t *hfoc, CalStruct *hcal) {
    if (hfoc == NULL || hcal == NULL || hcal->cal_state != CAL_STATE_LUT_POSTPROC_PENDING)
        return;

    const int n      = (int)LUT_NUM_SAMPLES;
    const int n_lut  = (int)ERROR_LUT_SIZE;
    const int window = (int)LUT_SAMPLES_PER_PPAIR;

    // 1. DC bias removal
    float ezero = 0.0f;
    for (int i = 0; i < n; i++) ezero += hcal->lut_raw[i];
    ezero /= (float)n;

    // 2. Resample onto LUT grid with moving-average filter (one pole-pair window)
    for (int i = 0; i < n_lut; i++) {
        int center = (int)((float)i * (float)n / (float)n_lut);
        float avg = 0.0f;
        for (int j = -window / 2; j < window / 2; j++) {
            int idx = center + j;
            if (idx < 0)       idx += n;
            else if (idx >= n) idx -= n;
            avg += hcal->lut_raw[idx];
        }
        hfoc->angle_sensor.encd_error_comp[i] = avg / (float)window - ezero;
    }

    hfoc->angle_sensor.lut_ready = 1;
    hfoc->control_mode           = ENCODER_MODE;
    hcal->cal_state              = CAL_STATE_IDLE;
}
