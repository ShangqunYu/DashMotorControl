/*
 * foc_calibration.c
 *
 * Phase 0 – Phase order and pole pair detection:
 *   Lock rotor at e=0 for 1 s, then rotate CW through one electrical cycle (2π).
 *   Measure raw MA732 angle delta to compute:
 *     pole_pairs = round(2π / |delta|)
 *     direction  = NORMAL if delta > 0, REVERSE if delta < 0
 *
 * Phase 1 – LUT sweep (Ben Katz method):
 *   Re-settle at e=0 for 1 s, then rotate CW then CCW at W_CAL.
 *   For reversed motors the voltage angle is negated so the rotor always
 *   sweeps in the mechanically positive direction.
 *   CW and CCW samples are averaged to cancel friction/cogging bias.
 *
 * Phase 2 – post-processing (call foc_cal_lut_postprocess from main loop):
 *   ezero = mean(lut_raw) = electrical zero offset.
 *   LUT stores residual nonlinearity (errors − ezero).
 */

#include "foc_calibration.h"
#include "foc.h"
#include "main.h"
#include "user_config.h"
#include <math.h>
#include <stdio.h>

void foc_cal_encoder_misalignment_start(foc_t *hfoc, CalStruct *hcal) {
    if (hfoc == NULL || hcal == NULL) return;

    hfoc->angle_sensor.e_zero     = 0.0f;
    hfoc->angle_sensor.lut_ready  = 0;
    hfoc->angle_sensor.sensor_dir = NORMAL_DIR;

    hcal->detected_ppairs  = 0;
    hcal->lut_num_samples  = 0;
    hcal->lut_theta_ref    = 0.0f;
    hcal->cal_state        = CAL_STATE_PHASE_SETTLING;
    hcal->cal_start_time   = HAL_GetTick();

    open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
    printf("Calibration started: settling...\r\n");
}

void foc_cal_encoder_misalignment_update(foc_t *hfoc, CalStruct *hcal, float Ts) {
    if (hfoc == NULL || hcal == NULL) return;

    // ── Phase 0a: hold at e=0 until rotor stops moving ───────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_SETTLING) {
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->phase_raw_start = DEG_TO_RAD(hfoc->angle_sensor.ma732.angle_filtered);
            hcal->lut_theta_ref   = 0.0f;
            hcal->cal_state       = CAL_STATE_PHASE_DETECT;
        }
        return;
    }

    // ── Phase 0b: rotate one electrical cycle CW ────────────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_DETECT) {
        hcal->lut_theta_ref += W_CAL * Ts;
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, hcal->lut_theta_ref);

        if (hcal->lut_theta_ref >= TWO_PI) {
            // Hold voltage at 2π so the rotor can catch up before we measure
            hcal->cal_start_time = HAL_GetTick();
            hcal->cal_state      = CAL_STATE_PHASE_MEASURE;
        }
        return;
    }

    // ── Phase 0c: hold at 2π, let rotor settle, then measure ────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_MEASURE) {
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, TWO_PI);

        if (HAL_GetTick() - hcal->cal_start_time >= 500) {
            float raw_end = DEG_TO_RAD(hfoc->angle_sensor.ma732.angle_filtered);
            float delta   = raw_end - hcal->phase_raw_start;
            while (delta >  PI) delta -= TWO_PI;
            while (delta < -PI) delta += TWO_PI;
            printf("Phase detection delta: %.4f rad\r\n", delta);

            uint8_t ppairs = (uint8_t)roundf(TWO_PI / fabsf(delta));
            if (ppairs < 1)          ppairs = 1;
            if (ppairs > PPAIRS_MAX) ppairs = (uint8_t)PPAIRS_MAX;

            hcal->detected_ppairs         = ppairs;
            hcal->lut_num_samples         = (uint16_t)((uint32_t)ppairs * LUT_SAMPLES_PER_PPAIR);
            hfoc->angle_sensor.pole_pairs = ppairs;
            PPAIRS = (float)ppairs;
            printf("Detected %d pole pairs\r\n", ppairs);

            if (delta < 0.0f) {
                hfoc->angle_sensor.sensor_dir = REVERSE_DIR;
                PHASE_ORDER = 1;
                printf("Phase order: reversed\r\n");
            } else {
                hfoc->angle_sensor.sensor_dir = NORMAL_DIR;
                PHASE_ORDER = 0;
                printf("Phase order: normal\r\n");
            }

            // Re-settle at e=0 before the LUT sweep
            hcal->lut_theta_ref  = 0.0f;
            hcal->cal_start_time = HAL_GetTick();
            hcal->cal_state      = CAL_STATE_LUT_SETTLING;
            open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
        }
        return;
    }

    // ── Phase 1a: re-settle before LUT sweep ────────────────────────────────
    if (hcal->cal_state == CAL_STATE_LUT_SETTLING) {
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->lut_theta_ref         = 0.0f;
            hcal->lut_next_sample_e_rad = 0.0f;
            hcal->lut_cal_idx           = 0;
            hcal->cal_state             = CAL_STATE_LUT_CW;
        }
        return;
    }

    // For reversed motors, negate the voltage angle so the rotor sweeps in
    // the mechanically positive (increasing MA732 angle) direction.
    uint8_t reversed = (hfoc->angle_sensor.sensor_dir == REVERSE_DIR);

    // ── Phase 1b: CW sweep ───────────────────────────────────────────────────
    if (hcal->cal_state == CAL_STATE_LUT_CW) {
        hcal->lut_theta_ref += W_CAL * Ts;
        float angle_out = reversed ? -hcal->lut_theta_ref : hcal->lut_theta_ref;
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, angle_out);

        if (hcal->lut_theta_ref >= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->lut_cal_idx;
            float ideal  = (float)idx * TWO_PI / (float)hcal->lut_num_samples;
            float err    = DEG_TO_RAD(hfoc->angle_sensor.ma732.angle_filtered) - ideal;
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;
            hcal->lut_raw[idx] = err;

            hcal->lut_next_sample_e_rad += TWO_PI / (float)LUT_SAMPLES_PER_PPAIR;

            if (++hcal->lut_cal_idx >= hcal->lut_num_samples) {
                hcal->lut_cal_idx           = hcal->lut_num_samples - 1u;
                hcal->lut_next_sample_e_rad = hcal->lut_theta_ref;
                hcal->cal_state             = CAL_STATE_LUT_CCW;
            }
        }
        return;
    }

    // ── Phase 1c: CCW sweep ──────────────────────────────────────────────────
    if (hcal->cal_state == CAL_STATE_LUT_CCW) {
        hcal->lut_theta_ref -= W_CAL * Ts;
        float angle_out = reversed ? -hcal->lut_theta_ref : hcal->lut_theta_ref;
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, angle_out);

        if (hcal->lut_theta_ref <= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->lut_cal_idx;
            float ideal  = (float)idx * TWO_PI / (float)hcal->lut_num_samples;
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

    if (hcal->cal_state == CAL_STATE_LUT_POSTPROC_PENDING) {
        open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
        return;
    }
}

// ── Post-processing (call from main loop, NOT from an ISR) ─────────────────
void foc_cal_lut_postprocess(foc_t *hfoc, CalStruct *hcal) {
    if (hfoc == NULL || hcal == NULL) return;
    if (hcal->cal_state != CAL_STATE_LUT_POSTPROC_PENDING) return;
    if (hcal->lut_num_samples == 0) return;

    const int n      = (int)hcal->lut_num_samples;
    const int n_lut  = (int)ERROR_LUT_SIZE;
    const int window = (int)LUT_SAMPLES_PER_PPAIR;

    float ezero = 0.0f;
    for (int i = 0; i < n; i++) ezero += hcal->lut_raw[i];
    ezero /= (float)n;

    hfoc->angle_sensor.e_zero = ezero;

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
