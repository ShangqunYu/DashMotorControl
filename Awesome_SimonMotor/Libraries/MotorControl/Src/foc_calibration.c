/*
 * foc_calibration.c
 *
 * Phase 0 – Phase order and pole pair detection:
 *   Lock rotor at e=0 for 1 s, then rotate CW through N_POLE_DETECT_CYCLES
 *   electrical cycles.  Using multiple cycles makes the mechanical displacement
 *   N× larger so rotor wobble has N× less influence on the pole-pair count.
 *     pole_pairs = round(N * 2π / |mech_displacement|)
 *     direction  = NORMAL if displacement > 0, REVERSE if displacement < 0
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

// Sweep this many electrical cycles for pole-pair detection.
// More cycles → larger mechanical displacement → less sensitivity to rotor wobble.
// Safe for pole_pairs >= ceil(N/2) (displacement < π per cycle limit).
#define N_POLE_DETECT_CYCLES 1

// Drive id=I_CAL, iq=0 at a forced electrical angle — used during calibration
// sweeps so the PI loop regulates current instead of applying raw voltage.
static void cal_force_current(foc_t *hfoc, float angle_rad, float Ts) {
    hfoc->angle_sensor.e_rad = angle_rad;
    hfoc->id_ref = I_CAL;
    hfoc->iq_ref = 0.0f;
    foc_current_control_update(hfoc, Ts);
}

void foc_cal_encoder_misalignment_start(foc_t *hfoc, CalStruct *hcal) {
    if (hfoc == NULL || hcal == NULL) return;

    hfoc->angle_sensor.e_zero     = 0.0f;
    hfoc->angle_sensor.lut_ready  = 0;
    hfoc->angle_sensor.sensor_dir = NORMAL_DIR;

    hcal->detected_ppairs  = 0;
    hcal->num_measurements_to_take_for_lut  = 0;
    hcal->cal_state        = CAL_STATE_PHASE_SETTLING;
    hcal->cal_start_time   = HAL_GetTick();
    
    printf("Calibration started: settling...\r\n");
}

void foc_cal_encoder_misalignment_update(foc_t *hfoc, CalStruct *hcal, float Ts) {
    if (hfoc == NULL || hcal == NULL) return;

    // ── Phase 0a: hold at e=0 until rotor stops moving ───────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_SETTLING) {
        cal_force_current(hfoc, 0.0f, Ts);
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->phase_raw_start = hfoc->angle_sensor.multi_rotor_rad;
            hcal->lut_theta_ref   = 0.0f;
            hcal->cal_state       = CAL_STATE_PHASE_DETECT;
        }
        return;
    }

    // ── Phase 0b: rotate one electrical cycle CW ────────────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_DETECT) {
        hcal->lut_theta_ref += W_CAL * Ts;
        cal_force_current(hfoc, hcal->lut_theta_ref, Ts);

        if (hcal->lut_theta_ref >= (float)N_POLE_DETECT_CYCLES * TWO_PI) {
            hcal->cal_start_time = HAL_GetTick();
            hcal->cal_state      = CAL_STATE_PHASE_MEASURE;
        }
        return;
    }

    // ── Phase 0c: hold at 2π, let rotor settle, then measure ────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_MEASURE) {
        cal_force_current(hfoc, (float)N_POLE_DETECT_CYCLES * TWO_PI, Ts);

        if (HAL_GetTick() - hcal->cal_start_time >= 500) {
            float delta = hfoc->angle_sensor.multi_rotor_rad - hcal->phase_raw_start;
            printf("Phase detection: total_mech=%.4f rad over %d e-cycles\r\n",
                   delta, N_POLE_DETECT_CYCLES);

            uint8_t ppairs = (uint8_t)roundf((float)N_POLE_DETECT_CYCLES * TWO_PI / fabsf(delta));
            if (ppairs > PPAIRS_MAX) {
                // TODO: throw some sort error here!!!!!
            }

            hcal->detected_ppairs         = ppairs;
            hcal->num_measurements_to_take_for_lut         = (uint16_t)(((uint32_t)ppairs) * MEASUREMENTS_PER_POLE_PAIR);
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
            hcal->cal_start_time = HAL_GetTick();
            hcal->cal_state      = CAL_STATE_LUT_SETTLING;
        }
        return;
    }

    // ── Phase 1a: re-settle before LUT sweep ────────────────────────────────
    if (hcal->cal_state == CAL_STATE_LUT_SETTLING) {
        cal_force_current(hfoc, 0.0f, Ts);
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->lut_theta_ref         = 0.0f;
            hcal->lut_next_sample_e_rad = 0.0f;
            hcal->curr_measurement_index           = 0;
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
        cal_force_current(hfoc, angle_out, Ts);

        if (hcal->lut_theta_ref >= hcal->lut_next_sample_e_rad) {
            uint16_t measurement_idx = hcal->curr_measurement_index;
            float ideal_rotor_angle_radians  = (float)measurement_idx * TWO_PI / (float)hcal->num_measurements_to_take_for_lut;
            float err    = hfoc->angle_sensor.ma732.angle_raw - ideal_rotor_angle_radians;

            // wrap to range [-180, 180]
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;
            hcal->all_measurements_for_lookup_table[measurement_idx] = err;
            
            // 2 pi * pp / num_measurements_to_take_for_lut = 2pi / MEASUREMENTS_PER_POLE_PAIR
            hcal->lut_next_sample_e_rad += TWO_PI / (float)MEASUREMENTS_PER_POLE_PAIR;


            // STATE TRANSITION
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
        hcal->lut_theta_ref -= W_CAL * Ts;
        float angle_out = reversed ? -hcal->lut_theta_ref : hcal->lut_theta_ref;
        cal_force_current(hfoc, angle_out, Ts);

        if (hcal->lut_theta_ref <= hcal->lut_next_sample_e_rad) {
            uint16_t measurement_index = hcal->curr_measurement_index;
            float ideal_rotor_angle_radians  = (float)measurement_index * TWO_PI / (float)hcal->num_measurements_to_take_for_lut;
            float err    = hfoc->angle_sensor.ma732.angle_raw - ideal_rotor_angle_radians;
            
            // wrap to range [-180, 180]
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;

            // average the new measurement with the previous measurement at this
            // position that we first got when traveling clockwise.
            // TODO: AVERAGING ANGULAR QUANTITIES IS SUSPECT WHEN AROUND THE DISCONTINUITY!!!!! COME BACK TO THIS!!!!!
            hcal->all_measurements_for_lookup_table[measurement_index] = (hcal->all_measurements_for_lookup_table[measurement_index] + err) * 0.5f;

            // 2 pi * pp / num_measurements_to_take_for_lut = 2pi / MEASUREMENTS_PER_POLE_PAIR
            hcal->lut_next_sample_e_rad -= TWO_PI / (float)MEASUREMENTS_PER_POLE_PAIR;

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
    const int window = (int)MEASUREMENTS_PER_POLE_PAIR;

    float ezero = 0.0f;
    for (int i = 0; i < n; i++) {
       ezero += hcal->all_measurements_for_lookup_table[i];
    }
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
