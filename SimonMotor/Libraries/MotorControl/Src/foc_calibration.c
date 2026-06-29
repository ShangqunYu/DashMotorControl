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


// Drive id=I_CAL, iq=0 at a forced electrical angle — used during calibration
// sweeps so the PI loop regulates current instead of applying raw voltage.
static void cal_force_current(foc_t *hfoc, float angle_rad) {
    hfoc->angle_sensor.e_rad = angle_rad;
    hfoc->id_ref = I_CAL;
    hfoc->iq_ref = 0.0f;
    foc_current_control_update(hfoc);
}

void foc_cal_encoder_misalignment_start(foc_t *hfoc, CalStruct *hcal) {

    hfoc->angle_sensor.e_zero               = 0.0f;
    hfoc->angle_sensor.sensor_dir           = NORMAL_DIR;
    hcal->num_measurements_to_take_for_lut  = 0;
    hfoc->angle_sensor.lut_ready            = 0;
    hcal->cal_state                         = CAL_STATE_PHASE_SETTLING;
    hcal->cal_start_time                    = HAL_GetTick();
    printf("Calibration started: settling...\r\n");
}

void foc_cal_encoder_misalignment_update(foc_t *hfoc, CalStruct *hcal) {

    // ── Phase 0a: hold at e=0 until rotor stops moving ───────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_SETTLING) {
        cal_force_current(hfoc, 0.0f);
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->phase_raw_start = hfoc->angle_sensor.multi_rotor_rad;
            hcal->lut_theta_ref   = 0.0f;
            hcal->cal_state       = CAL_STATE_PHASE_DETECT;
        }
        return;
    }

    // ── Phase 0b: rotate N_DETECT_ELECTRIC_CYCLE ────────────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_DETECT) {
        hcal->lut_theta_ref += W_CAL * FOC_TS;
        cal_force_current(hfoc, hcal->lut_theta_ref);

        if (hcal->lut_theta_ref >= (float)N_DETECT_ELECTRIC_CYCLE * TWO_PI) {
            hcal->cal_start_time = HAL_GetTick();
            hcal->cal_state      = CAL_STATE_PHASE_MEASURE;
        }
        return;
    }

    // ── Phase 0c: hold at N_DETECT_ELECTRIC_CYCLE, let rotor settle, then measure ────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_MEASURE) {
        cal_force_current(hfoc, (float)N_DETECT_ELECTRIC_CYCLE * TWO_PI);

        if (HAL_GetTick() - hcal->cal_start_time >= 500) {
            float delta   = hfoc->angle_sensor.multi_rotor_rad - hcal->phase_raw_start;
            printf("Phase detection: total_mech=%.4f rad over %d e-cycles\r\n",
                   delta, N_DETECT_ELECTRIC_CYCLE);

            hfoc->angle_sensor.pole_pairs = (uint8_t)roundf((float)N_DETECT_ELECTRIC_CYCLE * TWO_PI / fabsf(delta));

            hcal->num_measurements_to_take_for_lut         = (uint16_t)((uint32_t)hfoc->angle_sensor.pole_pairs * LUT_SAMPLES_PER_PPAIR);

            printf("Detected %d pole pairs\r\n", hfoc->angle_sensor.pole_pairs);

            if (delta < 0.0f) {
                hfoc->angle_sensor.sensor_dir = REVERSE_DIR;
                printf("Phase order: reversed\r\n");
            } else {
                hfoc->angle_sensor.sensor_dir = NORMAL_DIR;
                printf("Phase order: normal\r\n");
            }

            // Re-settle at e=0 before the LUT sweep
            hcal->cal_start_time = HAL_GetTick();
            hcal->cal_state      = CAL_STATE_LUT_SETTLING;
        }
        return;
    }

    // ── Phase 1a: re-settle before LUT sweep ────────────────────────────────
    if (hcal->cal_state == CAL_STATE_LUT_SETTLING) {
        cal_force_current(hfoc, 0.0f);
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->lut_theta_ref         = 0.0f;
            hcal->lut_next_sample_e_rad = 0.0f;
            hcal->curr_measurement_index           = 0;
            hcal->cal_state             = CAL_STATE_LUT_CW;
        }
        return;
    }

    // ── Phase 1b: CW sweep ───────────────────────────────────────────────────
    if (hcal->cal_state == CAL_STATE_LUT_CW) {
        hcal->lut_theta_ref += W_CAL * FOC_TS;
        // For reversed motors, negate the voltage angle so the rotor sweeps in
        // the mechanically positive (increasing MA732 angle) direction.
        float angle_out = hfoc->angle_sensor.sensor_dir ? -hcal->lut_theta_ref : hcal->lut_theta_ref;
        cal_force_current(hfoc, angle_out);

        if (hcal->lut_theta_ref >= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->curr_measurement_index;
            float ideal_rotor_angle_radians  = (float)idx * TWO_PI / (float)hcal->num_measurements_to_take_for_lut;
            float err    = hfoc->angle_sensor.raw_rad - ideal_rotor_angle_radians;
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;
            hcal->all_measurements_for_lookup_table[idx] = err;

            hcal->lut_next_sample_e_rad += TWO_PI / (float)LUT_SAMPLES_PER_PPAIR;

            if (++hcal->curr_measurement_index >= hcal->num_measurements_to_take_for_lut) {
                hcal->curr_measurement_index           = hcal->num_measurements_to_take_for_lut - 1u;
                hcal->lut_next_sample_e_rad = hcal->lut_theta_ref;
                hcal->cal_state             = CAL_STATE_LUT_CCW;
            }
        }
        return;
    }

    // ── Phase 1c: CCW sweep ──────────────────────────────────────────────────
    if (hcal->cal_state == CAL_STATE_LUT_CCW) {
        hcal->lut_theta_ref -= W_CAL * FOC_TS;
        float angle_out = hfoc->angle_sensor.sensor_dir ? -hcal->lut_theta_ref : hcal->lut_theta_ref;
        cal_force_current(hfoc, angle_out);

        if (hcal->lut_theta_ref <= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->curr_measurement_index;
            float ideal  = (float)idx * TWO_PI / (float)hcal->num_measurements_to_take_for_lut;
            float err    = hfoc->angle_sensor.raw_rad - ideal;
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;
            hcal->all_measurements_for_lookup_table[idx] = (hcal->all_measurements_for_lookup_table[idx] + err) * 0.5f;

            hcal->lut_next_sample_e_rad -= TWO_PI / (float)LUT_SAMPLES_PER_PPAIR;

            if (hcal->curr_measurement_index == 0) {
                open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
                hcal->cal_state = CAL_STATE_LUT_POSTPROC_PENDING;
            } else {
                hcal->curr_measurement_index--;
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
    if (hcal->num_measurements_to_take_for_lut == 0) return;

    const int n      = (int)hcal->num_measurements_to_take_for_lut;
    const int n_lut  = (int)ERROR_LUT_SIZE;
    const int window = (int)LUT_SAMPLES_PER_PPAIR;

    float ezero = 0.0f;
    for (int i = 0; i < n; i++) ezero += hcal->all_measurements_for_lookup_table[i];
    ezero /= (float)n;

    hfoc->angle_sensor.e_zero = ezero;

    for (int i = 0; i < n_lut; i++) {
        int center = (int)((float)i * (float)n / (float)n_lut);
        float avg = 0.0f;
        for (int j = -window / 2; j < window / 2; j++) {
            int idx = center + j;
            if (idx < 0)       idx += n;
            else if (idx >= n) idx -= n;
            avg += hcal->all_measurements_for_lookup_table[idx];
        }
        hfoc->angle_sensor.encd_error_comp[i] = avg / (float)window - ezero;
    }

    hfoc->angle_sensor.lut_ready = 1;
    hcal->cal_state              = CAL_STATE_IDLE;
}
