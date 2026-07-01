/*
 * foc_calibration.h
 */

#ifndef FOC_CALIBRATION_H_
#define FOC_CALIBRATION_H_

#include <stdint.h>
#include "hw_config.h"   /* LUT_SAMPLES_PER_PPAIR, PPAIRS_MAX, W_CAL */
#include "PMSM_motor.h"         /* foc_t */

/* ── LUT sizing ──────────────────────────────────────────────────────────── */
/* Buffer sized for worst-case pole pairs at compile time.
 * Actual sample count used during a sweep is hcal->lut_num_samples,
 * set after pole-pair detection. */
#define LUT_NUM_SAMPLES_MAX  ((uint32_t)(LUT_SAMPLES_PER_PPAIR) * (uint32_t)(PPAIRS_MAX))

/* ── Calibration state machine ───────────────────────────────────────────── */
typedef enum {
    CAL_STATE_IDLE,
    CAL_STATE_PHASE_SETTLING,         // lock at e=0, wait for rotor to settle
    CAL_STATE_PHASE_DETECT,           // rotate N_POLE_DETECT_CYCLES electrical cycles CW
    CAL_STATE_PHASE_MEASURE,          // hold voltage, wait for rotor to settle, then measure total displacement
    CAL_STATE_LUT_SETTLING,           // re-settle at e=0 before sweep
    CAL_STATE_LUT_CW,                 // rotating CW at W_CAL, collecting raw samples
    CAL_STATE_LUT_CCW,                // rotating CCW at W_CAL, averaging raw samples
    CAL_STATE_LUT_POSTPROC_PENDING,   // sweeps done; main loop calls foc_cal_lut_postprocess()
} cal_state_t;

/* ── Calibration data block ──────────────────────────────────────────────── */
typedef struct {
    cal_state_t cal_state;
    uint32_t    cal_start_time;

    /* Phase-order / pole-pair detection */
    float       phase_raw_start;     // raw MA732 angle (rad) at start of detection sweep

    /* LUT sweep */
    float       all_measurements_for_lookup_table[LUT_NUM_SAMPLES_MAX]; // CW/CCW-averaged raw error samples
    uint16_t    num_measurements_to_take_for_lut;              // actual samples = ppairs * LUT_SAMPLES_PER_PPAIR
    uint16_t    curr_measurement_index;
    float       lut_theta_ref;
    float       lut_next_sample_e_rad;
} CalStruct;

/* ── Public API ──────────────────────────────────────────────────────────── */
void foc_cal_encoder_misalignment_start(PMSM_motor *motor, CalStruct *hcal);
void cal_encoder_misalignment_update(PMSM_motor *motor, CalStruct *hcal);
void foc_cal_lut_postprocess(PMSM_motor *motor, CalStruct *hcal);
void foc_r_meas_update(PMSM_motor *motor);
void foc_l_meas_update(PMSM_motor *motor);
void inject_amp_and_measure_id_iq(PMSM_motor *motor, float vd_ref, float vq_ref, float *id, float *iq);

#endif /* FOC_CALIBRATION_H_ */