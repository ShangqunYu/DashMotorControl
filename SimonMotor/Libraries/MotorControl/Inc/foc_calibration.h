/*
 * foc_calibration.h
 *
 * Calibration types and API for encoder offset + nonlinearity LUT calibration.
 *
 * Include note
 * ------------
 * This file includes foc.h for foc_t. foc.h does not include this file,
 * so there is no circular dependency.
 */

#ifndef FOC_CALIBRATION_H_
#define FOC_CALIBRATION_H_

#include <stdint.h>
#include "hw_config.h"   /* LUT_SAMPLES_PER_PPAIR, POLE_PAIR, W_CAL */
#include "foc.h"         /* foc_t */

/* ── LUT sizing ──────────────────────────────────────────────────────────── */
/* ERROR_LUT_SIZE is defined in hw_config.h */
#define LUT_NUM_SAMPLES   ((uint32_t)(LUT_SAMPLES_PER_PPAIR) * (uint32_t)(POLE_PAIR))

/* ── Calibration state machine ───────────────────────────────────────────── */
typedef enum {
    CAL_STATE_IDLE,
    CAL_STATE_SETTLING,              // rotor lock at e=0 before offset sampling
    CAL_STATE_SAMPLING,              // collecting offset samples
    CAL_STATE_COMPLETE,              // offset done, start LUT sweep
    CAL_STATE_LUT_SETTLING,          // lock at e=0 for T1 before sweep
    CAL_STATE_LUT_CW,                // rotating CW at W_CAL, collecting raw samples
    CAL_STATE_LUT_CCW,               // rotating CCW at W_CAL, averaging raw samples
    CAL_STATE_LUT_POSTPROC_PENDING,  // sweeps done; main loop calls foc_cal_lut_postprocess()
} cal_state_t;

/* ── Calibration data block ──────────────────────────────────────────────── */
typedef struct {
    cal_state_t cal_state;
    uint32_t    cal_start_time;
    uint32_t    cal_sample_count;
    float       cal_rad_offset_sum;

    float       lut_raw[LUT_NUM_SAMPLES];  // intermediate CW/CCW-averaged raw samples
    uint16_t    lut_cal_idx;               // current sample index during sweep
    float       lut_theta_ref;             // running electrical angle command
    float       lut_next_sample_e_rad;     // next electrical angle sample trigger
} CalStruct;

/* ── Public API ──────────────────────────────────────────────────────────── */

// Start a full calibration sequence (offset + LUT sweep).
// Call once; then call foc_cal_encoder_misalignment_update() every FOC cycle.
void foc_cal_encoder_misalignment_start(foc_t *hfoc, CalStruct *hcal);

// Run one step of the calibration state machine.
// Must be called every FOC cycle while control_mode == CALIBRATION_MODE.
void foc_cal_encoder_misalignment_update(foc_t *hfoc, CalStruct *hcal, float Ts);

// Post-process raw sweep data into the correction LUT.
// Call from the main loop (NOT from an ISR) when
// hcal->cal_state == CAL_STATE_LUT_POSTPROC_PENDING.
void foc_cal_lut_postprocess(foc_t *hfoc, CalStruct *hcal);

#endif /* FOC_CALIBRATION_H_ */
