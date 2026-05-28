/*
 * foc_calibration.h
 */

#ifndef FOC_CALIBRATION_H_
#define FOC_CALIBRATION_H_

#include <stdint.h>
#include "hw_config.h"   /* LUT_SAMPLES_PER_PPAIR, PPAIRS_MAX, W_CAL */
#include "foc.h"         /* foc_t */

/* ── LUT sizing ──────────────────────────────────────────────────────────── */
/* Buffer sized for worst-case pole pairs at compile time.
 * Actual sample count used during a sweep is hcal->lut_num_samples,
 * set after pole-pair detection. */
#define LUT_NUM_SAMPLES_MAX  ((uint32_t)(LUT_SAMPLES_PER_PPAIR) * (uint32_t)(PPAIRS_MAX))

/* ── Calibration state machine ───────────────────────────────────────────── */
typedef enum {
    CAL_STATE_IDLE,
    CAL_STATE_PHASE_SETTLING,         // lock at e=0, wait for rotor to settle
    CAL_STATE_PHASE_DETECT,           // rotate one electrical cycle CW
    CAL_STATE_PHASE_MEASURE,          // hold voltage at 2π, wait for rotor to catch up, then measure
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
    uint8_t     detected_ppairs;     // pole pairs measured; 0 = not yet detected

    /* LUT sweep */
    float       lut_raw[LUT_NUM_SAMPLES_MAX]; // CW/CCW-averaged raw error samples
    uint16_t    lut_num_samples;              // actual samples = detected_ppairs * LUT_SAMPLES_PER_PPAIR
    uint16_t    lut_cal_idx;
    float       lut_theta_ref;
    float       lut_next_sample_e_rad;
} CalStruct;

/* ── Public API ──────────────────────────────────────────────────────────── */
void foc_cal_encoder_misalignment_start(foc_t *hfoc, CalStruct *hcal);
void foc_cal_encoder_misalignment_update(foc_t *hfoc, CalStruct *hcal, float Ts);
void foc_cal_lut_postprocess(foc_t *hfoc, CalStruct *hcal);

#endif /* FOC_CALIBRATION_H_ */
