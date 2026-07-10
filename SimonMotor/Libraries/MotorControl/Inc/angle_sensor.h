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
#include "stm32f4xx_hal.h"
#include "hw_config.h"  



float MA732_get_rad();

/* ── Sensor direction ────────────────────────────────────────────────────── */
typedef enum {
    NORMAL_DIR,
    REVERSE_DIR
} dir_mode_t;

/* ── Velocity observer constants ─────────────────────────────────────────
 * Luenberger observer tuned for bandwidth OBS_BW_HZ at 40 kHz.
 * To retune: p = exp(-2π·fc·Ts), L1 = 2(1-p), L2 = (1-p)²/Ts
 */
#define OBS_BW_HZ     100.0f
#define OBS_L1        0.0311  // 2·(1−p),      fc=100 Hz, Ts=25 µs
#define OBS_L2        9.70f    // (1−p)²/Ts,    fc=100 Hz, Ts=25 µs
#define VEL_ZERO_THRESH   0.01f  // rad/s — clamp to 0 below this 

/* ── Angle / velocity state ──────────────────────────────────────────────── */
typedef struct {

    float raw_rad;          // raw radiance right from the encoder reading
    float single_rotor_rad;      // user position single turn (LUT-corrected, relative to m_zero)
    float single_rotor_rad_raw;  // user position before LUT correction (same reference, for comparison)
    float multi_rotor_rad;  // multiturn mechanical angle (LUT-corrected, relative to m_zero)
    float mech_angle_rad;   // mechanical agnle for the output shaft
    float e_zero;           // electrical zero (rad): encoder reading when e=0, from calibration
    float m_zero;           // mechanical zero (rad): user-defined position reference
    float e_rad;            // electric angle latched each FOC cycle
    int turns;              // cumulative turn count for multi-turn support

    // velocity observer related state
    float rotor_vel;        // rotor velocity (rad/s, direction-corrected)
    float mech_angle_vel;   // mechanical angular velocity after the gear ratio
    float pos_hat;          // observer: estimated position
    float vel_hat;          // observer: estimated velocity
    uint8_t obs_ready;      // 1 once observer has been initialised


    uint8_t pole_pairs, first_sample;
    dir_mode_t sensor_dir;

    /* Encoder nonlinearity correction LUT (populated by angle_sensor_load_lut) */
    float   encd_error_comp[ERROR_LUT_SIZE];
    uint8_t lut_ready;   // 1 once LUT has been loaded and should be applied
    uint16_t spi_rx_buffer;   // raw 16-bit encoder frame; DMA target, must stay halfword-aligned
    uint8_t encd_get_val_flag;
} AngleSensor_t;

float sensor_get_rad(AngleSensor_t *sensor);
int sensor_start(AngleSensor_t *sensor);


/* ── Public API ──────────────────────────────────────────────────────────── */

// Initialise scalar fields; MA732 must be configured separately via MA732_config().
AngleSensor_t angle_sensor_init();


// Copy a processed correction LUT into the sensor and mark it active.
// Call this from the main loop after foc_cal_lut_postprocess() finishes.
void angle_sensor_load_lut(AngleSensor_t *sensor,
                           const float   *lut,
                           uint16_t       lut_size);


// update the angle sensor related data.
void angle_sensor_update(AngleSensor_t *sensor);

// Compute mechanical + electrical angles from the MA732 reading via polling.
void angle_sensor_update_position(AngleSensor_t *sensor);

// Compute mechanical velocity in rad/s from the delta of s_rotor_rad.
// Call once per FOC cycle (Ts = FOC sample period in seconds).
void angle_sensor_update_velocity(AngleSensor_t *sensor);

// Capture the current position as the mechanical zero.
// After this call s_rotor_rad == 0; save m_zero to flash from the main loop.
void angle_sensor_set_m_zero(AngleSensor_t *sensor);






#endif /* ANGLE_SENSOR_INC_ANGLE_SENSOR_H_ */
