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
#include "PMSM_motor.h"
#include "foc.h"
#include "main.h"
#include "user_config.h"
#include <math.h>
#include <stdio.h>


// Drive id=I_CAL, iq=0 at a forced electrical angle — used during calibration
// sweeps so the PI loop regulates current instead of applying raw voltage.
static void cal_force_current(PMSM_motor *motor, float angle_rad) {
    motor->angle_sensor.e_rad = angle_rad;
    motor->id_ref = I_CAL;
    motor->iq_ref = 0.0f;
    current_control_update(motor);
}

void foc_cal_encoder_misalignment_start(PMSM_motor *motor, CalStruct *hcal) {

    motor->angle_sensor.e_zero               = 0.0f;
    motor->angle_sensor.sensor_dir           = NORMAL_DIR;
    hcal->num_measurements_to_take_for_lut  = 0;
    motor->angle_sensor.lut_ready            = 0;
    hcal->cal_state                         = CAL_STATE_PHASE_SETTLING;
    hcal->cal_start_time                    = HAL_GetTick();
    printf("Calibration started: settling...\r\n");
}

void cal_encoder_misalignment_update(PMSM_motor *motor, CalStruct *hcal) {

    // ── Phase 0a: hold at e=0 until rotor stops moving ───────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_SETTLING) {
        cal_force_current(motor, 0.0f);
        if (HAL_GetTick() - hcal->cal_start_time >= 1000) {
            hcal->phase_raw_start = motor->angle_sensor.multi_rotor_rad;
            hcal->lut_theta_ref   = 0.0f;
            hcal->cal_state       = CAL_STATE_PHASE_DETECT;
        }
        return;
    }

    // ── Phase 0b: rotate N_DETECT_ELECTRIC_CYCLE ────────────────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_DETECT) {
        hcal->lut_theta_ref += W_CAL * FOC_TS;
        cal_force_current(motor, hcal->lut_theta_ref);

        if (hcal->lut_theta_ref >= (float)N_DETECT_ELECTRIC_CYCLE * TWO_PI) {
            hcal->cal_start_time = HAL_GetTick();
            hcal->cal_state      = CAL_STATE_PHASE_MEASURE;
        }
        return;
    }

    // ── Phase 0c: hold at N_DETECT_ELECTRIC_CYCLE, let rotor settle, then measure ────────────────
    if (hcal->cal_state == CAL_STATE_PHASE_MEASURE) {
        cal_force_current(motor, (float)N_DETECT_ELECTRIC_CYCLE * TWO_PI);

        if (HAL_GetTick() - hcal->cal_start_time >= 500) {
            float delta   = motor->angle_sensor.multi_rotor_rad - hcal->phase_raw_start;
            printf("Phase detection: total_mech=%.4f rad over %d e-cycles\r\n",
                   delta, N_DETECT_ELECTRIC_CYCLE);

            motor->angle_sensor.pole_pairs = (uint8_t)roundf((float)N_DETECT_ELECTRIC_CYCLE * TWO_PI / fabsf(delta));

            hcal->num_measurements_to_take_for_lut         = (uint16_t)((uint32_t)motor->angle_sensor.pole_pairs * LUT_SAMPLES_PER_PPAIR);

            printf("Detected %d pole pairs\r\n", motor->angle_sensor.pole_pairs);

            if (delta < 0.0f) {
                motor->angle_sensor.sensor_dir = REVERSE_DIR;
                printf("Phase order: reversed\r\n");
            } else {
                motor->angle_sensor.sensor_dir = NORMAL_DIR;
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
        cal_force_current(motor, 0.0f);
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
        float angle_out = motor->angle_sensor.sensor_dir ? -hcal->lut_theta_ref : hcal->lut_theta_ref;
        cal_force_current(motor, angle_out);

        if (hcal->lut_theta_ref >= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->curr_measurement_index;
            float ideal_rotor_angle_radians  = (float)idx * TWO_PI / (float)hcal->num_measurements_to_take_for_lut;
            float err    = motor->angle_sensor.raw_rad - ideal_rotor_angle_radians;
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
        float angle_out = motor->angle_sensor.sensor_dir ? -hcal->lut_theta_ref : hcal->lut_theta_ref;
        cal_force_current(motor, angle_out);

        if (hcal->lut_theta_ref <= hcal->lut_next_sample_e_rad) {
            uint16_t idx = hcal->curr_measurement_index;
            float ideal  = (float)idx * TWO_PI / (float)hcal->num_measurements_to_take_for_lut;
            float err    = motor->angle_sensor.raw_rad - ideal;
            while (err >  PI) err -= TWO_PI;
            while (err < -PI) err += TWO_PI;
            hcal->all_measurements_for_lookup_table[idx] = (hcal->all_measurements_for_lookup_table[idx] + err) * 0.5f;

            hcal->lut_next_sample_e_rad -= TWO_PI / (float)LUT_SAMPLES_PER_PPAIR;

            if (hcal->curr_measurement_index == 0) {
                open_loop_voltage_control(motor, 0.0f, 0.0f, 0.0f);
                hcal->cal_state = CAL_STATE_LUT_POSTPROC_PENDING;
            } else {
                hcal->curr_measurement_index--;
            }
        }
        return;
    }

    if (hcal->cal_state == CAL_STATE_LUT_POSTPROC_PENDING) {
        open_loop_voltage_control(motor, 0.0f, 0.0f, 0.0f);
        return;
    }
}

// ── Post-processing (call from main loop, NOT from an ISR) ─────────────────
void foc_cal_lut_postprocess(PMSM_motor *motor, CalStruct *hcal) {
    if (motor == NULL || hcal == NULL) return;
    if (hcal->cal_state != CAL_STATE_LUT_POSTPROC_PENDING) return;
    if (hcal->num_measurements_to_take_for_lut == 0) return;

    const int n      = (int)hcal->num_measurements_to_take_for_lut;
    const int n_lut  = (int)ERROR_LUT_SIZE;
    const int window = (int)LUT_SAMPLES_PER_PPAIR;

    float ezero = 0.0f;
    for (int i = 0; i < n; i++) ezero += hcal->all_measurements_for_lookup_table[i];
    ezero /= (float)n;

    motor->angle_sensor.e_zero = ezero;

    for (int i = 0; i < n_lut; i++) {
        int center = (int)((float)i * (float)n / (float)n_lut);
        float avg = 0.0f;
        for (int j = -window / 2; j < window / 2; j++) {
            int idx = center + j;
            if (idx < 0)       idx += n;
            else if (idx >= n) idx -= n;
            avg += hcal->all_measurements_for_lookup_table[idx];
        }
        motor->angle_sensor.encd_error_comp[i] = avg / (float)window - ezero;
    }

    motor->angle_sensor.lut_ready = 1;
    hcal->cal_state              = CAL_STATE_IDLE;
}


/* ── R measurement ───────────────────────────────────────────────────────────
 * Applies a fixed Vd at θ = 0, waits for steady state, then averages Id.
 * Rs = Vd / mean(Id).  Total time: (WARMUP + SAMPLES) / 40000 Hz ≈ 25 ms.
 */
#define R_MEAS_WARMUP   5000u   // cycles for L*dI/dt transient to die out
#define R_MEAS_SAMPLES  500u   // averaging window

static float r_meas_id_buf[R_MEAS_SAMPLES];

void foc_r_meas_update(PMSM_motor *motor)
{
    float id, iq;
    inject_amp_and_measure_id_iq(motor, motor->meas_inj_amp, 0.0f, &id, &iq);

    if (motor->meas_inj_n >= R_MEAS_WARMUP) {
        uint32_t idx = motor->meas_inj_n - R_MEAS_WARMUP;
        if (idx < R_MEAS_SAMPLES) {
            r_meas_id_buf[idx] = id;
        }
    }

    motor->meas_inj_n++;

    if (motor->meas_inj_n >= R_MEAS_WARMUP + R_MEAS_SAMPLES) {
        float mean_id = 0.0f;
        for (uint32_t i = 0; i < R_MEAS_SAMPLES; i++) {
            mean_id += r_meas_id_buf[i];
        }
        mean_id /= (float)R_MEAS_SAMPLES;
        if (fabsf(mean_id) > 0.01f) {
            motor->Rs = motor->meas_inj_amp / mean_id;
        }
        motor->meas_inj_n = 0;
        motor->meas_done  = 1;
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

void foc_l_meas_update(PMSM_motor *motor)
{
    /* Pre-alignment: hold DC Vd for L_MEAS_ALIGN cycles to lock the rotor at
     * θ = 0 before AC injection starts.  meas_inj_n counts globally so both
     * the warmup and the DFT collection happen AFTER alignment completes. */
#define L_MEAS_ALIGN  800u   // 20 ms — long enough for rotor to settle at θ=0

    if (motor->meas_inj_n < L_MEAS_ALIGN) {
        open_loop_voltage_control(motor, motor->meas_inj_amp * 0.5f, 0.0f, 0.0f);
        motor->meas_inj_n++;
        return;
    }
    /* Offset the cycle counter so warmup/DFT indices start from 0 after align */
    const uint32_t n_ac = motor->meas_inj_n - L_MEAS_ALIGN;

    /* Injection angle for this cycle — same for both applying V and DFT reference */
    const float angle = L_MEAS_OMEGA_STEP * (float)n_ac;
    const float sin_a = fast_sin(angle);
    const float cos_a = fast_cos(angle);
    const float v_inj = motor->meas_inj_amp * sin_a;

    /* d-axis for Ld phase, q-axis for Lq phase */
    const float vd_ref = (motor->l_meas_phase == 0) ? v_inj : 0.0f;
    const float vq_ref = (motor->l_meas_phase == 0) ? 0.0f  : v_inj;

    float id, iq;
    inject_amp_and_measure_id_iq(motor, vd_ref, vq_ref, &id, &iq);

    /* Accumulate DFT after warmup */
    if (n_ac >= L_MEAS_WARMUP) {
        const float v = (motor->l_meas_phase == 0) ? vd_ref : vq_ref;
        const float i = (motor->l_meas_phase == 0) ? id     : iq;
        motor->l_meas_Vc += v * cos_a;
        motor->l_meas_Vs += v * sin_a;
        motor->l_meas_Ic += i * cos_a;
        motor->l_meas_Is += i * sin_a;
    }

    motor->meas_inj_n++;

    if (n_ac + 1 >= L_MEAS_WARMUP + L_MEAS_SAMPLES) {
        /* Normalise (2/N DFT convention gives peak amplitudes) */
        const float norm  = 2.0f / (float)L_MEAS_SAMPLES;
        const float Vc    = motor->l_meas_Vc * norm;
        const float Vs    = motor->l_meas_Vs * norm;
        const float Ic    = motor->l_meas_Ic * norm;
        const float Is    = motor->l_meas_Is * norm;
        const float V_mag = sqrtf(Vc*Vc + Vs*Vs);
        const float I_mag = sqrtf(Ic*Ic + Is*Is);

        if (I_mag > 0.001f) {
            const float phi   = atan2f(Vs, Vc) - atan2f(Is, Ic);
            const float L_est = fabsf(V_mag * sinf(phi) / I_mag) / L_MEAS_OMEGA_RADS;
            if (motor->l_meas_phase == 0) motor->Ld = L_est;
            else                          motor->Lq = L_est;
        }

        /* Reset accumulators; skip re-alignment for the second phase since the
         * rotor is already locked from the Ld injection just before. */
        motor->l_meas_Vc = motor->l_meas_Vs = 0.0f;
        motor->l_meas_Ic = motor->l_meas_Is = 0.0f;
        motor->meas_inj_n = L_MEAS_ALIGN;   /* jump past alignment phase */

        if (motor->l_meas_phase == 1) {
            motor->l_meas_phase = 0;   /* proceed to Lq */
        } else {
            motor->meas_done = 1;      /* both Ld and Lq done */
        }
    }
}


void inject_amp_and_measure_id_iq(PMSM_motor *motor, float vd_ref, float vq_ref, float *id, float *iq) {
    open_loop_voltage_control(motor, vd_ref, vq_ref, 0.0f);

    CurrentSensor_update(&motor->current_sensor);
    float sin_theta, cos_theta;
    pre_calc_sin_cos(0.0f, &sin_theta, &cos_theta);
    clarke_park_transform(motor->current_sensor.ia, motor->current_sensor.ib, motor->current_sensor.ic,
                          sin_theta, cos_theta, id, iq);
}