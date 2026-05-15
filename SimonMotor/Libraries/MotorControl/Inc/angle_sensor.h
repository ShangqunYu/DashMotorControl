/*
 * angle_sensor.h
 *
 * AngleSensor_t – wraps MA732 and all angle/velocity state that the FOC
 * algorithm consumes.  Keeps magnetic encoder concerns out of foc_t.
 *
 * The encoder nonlinearity correction LUT lives here so that angle_sensor.h
 * does not need to depend on foc_calibration.h.  After calibration is
 * complete, call angle_sensor_load_lut() to copy the processed LUT in.
 */

#ifndef ANGLE_SENSOR_INC_ANGLE_SENSOR_H_
#define ANGLE_SENSOR_INC_ANGLE_SENSOR_H_

#include <stdint.h>
#include "MA732.h"
#include "hw_config.h"   /* ERROR_LUT_SIZE, POLE_PAIR */

/* ── Sensor direction ────────────────────────────────────────────────────── */
typedef enum {
    NORMAL_DIR,
    REVERSE_DIR
} dir_mode_t;

/* ── Velocity filter constants ───────────────────────────────────────────── */
#define VEL_FILTER_ALPHA  0.71539f   // IIR coefficient (same as MA732 RPM filter)
#define MAX_VEL_JUMP      5.24f      // rad/s spike rejection threshold (~50 RPM)
#define VEL_ZERO_THRESH   0.01f      // rad/s — clamp to 0 below this (~0.1 RPM)

/* ── Angle / velocity state ──────────────────────────────────────────────── */
typedef struct {
    MA732_t ma732;

    float m_angle_rad;       // user position (LUT-corrected, relative to m_zero)
    float m_angle_rad_raw;   // user position before LUT correction (same reference, for comparison)
    float e_angle_rad;       // un-normalised electric angle
    float e_angle_rad_comp;  // normalised electric angle (used by current control)
    float e_zero;            // electrical zero (rad): encoder reading when e=0, from calibration
    float m_zero;            // mechanical zero (rad): user-defined position reference
    float e_rad;             // electric angle latched each FOC cycle
    float last_e_rad;

    float actual_vel;        // mechanical velocity (rad/s, direction-corrected)
    float prev_m_angle_rad;  // previous m_angle_rad for delta computation
    float prev_vel;          // previous instantaneous velocity (spike rejection)
    float filtered_vel;      // IIR filter state

    uint8_t pole_pairs;
    dir_mode_t sensor_dir;

    /* Encoder nonlinearity correction LUT (populated by angle_sensor_load_lut) */
    float   encd_error_comp[ERROR_LUT_SIZE];
    uint8_t lut_ready;   // 1 once LUT has been loaded and should be applied
} AngleSensor_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

// Initialise scalar fields; MA732 must be configured separately via MA732_config().
void angle_sensor_init(AngleSensor_t *sensor,
                       uint8_t        pole_pairs,
                       float          e_zero_rad,
                       dir_mode_t     sensor_dir);

// Copy a processed correction LUT into the sensor and mark it active.
// Call this from the main loop after foc_cal_lut_postprocess() finishes.
void angle_sensor_load_lut(AngleSensor_t *sensor,
                           const float   *lut,
                           uint16_t       lut_size);

// Compute mechanical + electrical angles from the latest MA732 reading.
// Call from the SPI-complete ISR (HAL_SPI_TxRxCpltCallback).
void angle_sensor_update(AngleSensor_t *sensor);

// Compute mechanical velocity in rad/s from the delta of m_angle_rad.
// Call once per FOC cycle (Ts = FOC sample period in seconds).
void angle_sensor_update_velocity(AngleSensor_t *sensor, float Ts);

// Capture the current position as the mechanical zero.
// After this call m_angle_rad == 0; save m_zero to flash from the main loop.
void angle_sensor_set_m_zero(AngleSensor_t *sensor);

#endif /* ANGLE_SENSOR_INC_ANGLE_SENSOR_H_ */
