/*
 * foc_calibration.c
 *
 * Encoder nonlinearity LUT calibration (Ben Katz method).
 *
 * Phase 1 – LUT sweep:
 *   Lock rotor at e=0 for 1 s, then rotate CW then CCW at W_CAL (electrical
 *   rad/s).  CW and CCW samples are averaged to cancel friction bias.
 *
 * Phase 2 – post-processing (call foc_cal_lut_postprocess from main loop):
 *   Compute ezero = mean(all raw errors).  This is the electrical zero —
 *   the DC offset between the encoder's natural zero and e=0.  It becomes
 *   e_zero so that m_angle_rad = 0 aligns with e=0 at runtime.
 *   The LUT stores only the residual nonlinearity (errors - ezero), which
 *   is the same approach as Ben Katz's calibration.
 *
 * Note: mechanical zero (user-facing position reference) is independent and
 * can be zeroed separately after calibration.
 */

#include "foc_calibration.h"
#include "foc.h"
#include "main.h"

void foc_cal_encoder_misalignment_start(foc_t *hfoc, CalStruct *hcal) {
    if (hfoc == NULL || hcal == NULL) return;

    // Clear any previous offset — sweep samples raw angle so offset must be 0
    hfoc->angle_sensor.e_zero = 0.0f;
    hfoc->angle_sensor.lut_ready      = 0;

    hcal->cal_state    = CAL_STATE_LUT_SETTLING;
    hcal->cal_start_time = HAL_GetTick();

    open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
}

void foc_cal_encoder_misalignment_update(foc_t *hfoc, CalStruct *hcal, float Ts) {
    if (hfoc == NULL || hcal == NULL) return;

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
            float err    = DEG_TO_RAD(hfoc->angle_sensor.ma732.angle_filtered) - ideal;
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
            float err    = DEG_TO_RAD(hfoc->angle_sensor.ma732.angle_filtered) - ideal;
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
// ezero = mean(lut_raw) is the electrical zero — the DC offset between
// the encoder's natural zero and electrical angle 0.  It is stored as
// e_zero so runtime angle reading is anchored to e=0.
// The LUT then holds only the residual nonlinearity (errors - ezero).
void foc_cal_lut_postprocess(foc_t *hfoc, CalStruct *hcal) {
    if (hfoc == NULL || hcal == NULL || hcal->cal_state != CAL_STATE_LUT_POSTPROC_PENDING)
        return;

    const int n      = (int)LUT_NUM_SAMPLES;
    const int n_lut  = (int)ERROR_LUT_SIZE;
    const int window = (int)LUT_SAMPLES_PER_PPAIR;

    // 1. ezero = mean of all errors = electrical zero offset (rad)
    float ezero = 0.0f;
    for (int i = 0; i < n; i++) ezero += hcal->lut_raw[i];
    ezero /= (float)n;

    // Store as e_zero:
    hfoc->angle_sensor.e_zero = ezero;

    // 2. Resample onto LUT grid with moving-average filter (one pole-pair window)
    //    Subtracting ezero leaves only nonlinearity.
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
