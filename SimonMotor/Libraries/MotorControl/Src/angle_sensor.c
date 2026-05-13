/*
 * angle_sensor.c
 */

#include "angle_sensor.h"
#include "FOC_math.h"
#include <string.h>   /* memcpy */
#include <math.h>     /* floorf, fabsf, copysignf, fminf */

void angle_sensor_init(AngleSensor_t *sensor,
                       uint8_t        pole_pairs,
                       float          m_rad_offset,
                       dir_mode_t     sensor_dir)
{
    if (sensor == NULL) return;

    sensor->pole_pairs       = pole_pairs;
    sensor->m_angle_offset   = m_rad_offset;
    sensor->sensor_dir       = sensor_dir;
    sensor->m_angle_rad      = 0.0f;
    sensor->m_angle_rad_raw  = 0.0f;
    sensor->e_angle_rad      = 0.0f;
    sensor->e_angle_rad_comp = 0.0f;
    sensor->e_rad            = 0.0f;
    sensor->last_e_rad       = 0.0f;
    sensor->actual_vel       = 0.0f;
    sensor->prev_m_angle_rad = 0.0f;
    sensor->prev_vel         = 0.0f;
    sensor->filtered_vel     = 0.0f;
    sensor->lut_ready        = 0U;
}

void angle_sensor_load_lut(AngleSensor_t *sensor,
                           const float   *lut,
                           uint16_t       lut_size)
{
    if (sensor == NULL || lut == NULL) return;

    uint16_t copy_size = lut_size < ERROR_LUT_SIZE ? lut_size : ERROR_LUT_SIZE;
    memcpy(sensor->encd_error_comp, lut, copy_size * sizeof(float));
    sensor->lut_ready = 1U;
}

void angle_sensor_update(AngleSensor_t *sensor)
{
    if (sensor == NULL) return;

    float angle_deg = MA732_get_degree(&sensor->ma732);

    // Offset-corrected mechanical angle
    sensor->m_angle_rad = DEG_TO_RAD(angle_deg) - sensor->m_angle_offset;
    norm_angle_rad(&sensor->m_angle_rad);
    sensor->m_angle_rad_raw = sensor->m_angle_rad;  // snapshot before LUT correction

    // Apply encoder nonlinearity correction via interpolated LUT
    if (sensor->lut_ready) {
        float lut_idx_f = (sensor->m_angle_rad / TWO_PI) * (float)ERROR_LUT_SIZE;
        int   idx0      = (int)lut_idx_f;
        int   idx1      = (idx0 + 1) % (int)ERROR_LUT_SIZE;
        float frac      = lut_idx_f - (float)idx0;
        float correction = sensor->encd_error_comp[idx0] * (1.0f - frac)
                         + sensor->encd_error_comp[idx1] * frac;
        sensor->m_angle_rad -= correction;
        norm_angle_rad(&sensor->m_angle_rad);
    }

    // Electric angle (un-normalised)
    float e_rad = sensor->m_angle_rad * (float)sensor->pole_pairs;

    if (sensor->sensor_dir == REVERSE_DIR) {
        e_rad = TWO_PI - e_rad;
    }

    sensor->e_angle_rad = e_rad;

    // Normalised electric angle used by the current controller
    norm_angle_rad(&e_rad);
    sensor->e_angle_rad_comp = e_rad;

    MA732_set_val_flag();
}

void angle_sensor_update_velocity(AngleSensor_t *sensor, float Ts)
{
    if (sensor == NULL || Ts <= 0.0f) return;

    // Angular delta in rad, wrapped to [-π, π] to handle 0/2π boundary
    float delta = sensor->m_angle_rad - sensor->prev_m_angle_rad;
    delta -= TWO_PI * floorf((delta + PI) / TWO_PI);
    sensor->prev_m_angle_rad = sensor->m_angle_rad;

    // Instantaneous velocity in rad/s
    float vel_instant = delta / Ts;

    // Two-stage spike rejection (same logic as MA732 RPM filter)
    float vel_delta = vel_instant - sensor->prev_vel;
    float abs_delta = fabsf(vel_delta);
    if (abs_delta > MAX_VEL_JUMP) {
        float limited = copysignf(fminf(abs_delta * 0.5f, MAX_VEL_JUMP), vel_delta);
        vel_instant = sensor->prev_vel + limited;
    }

    // IIR low-pass filter
    float filtered = sensor->filtered_vel * (1.0f - VEL_FILTER_ALPHA)
                   + vel_instant          * VEL_FILTER_ALPHA;

    // Clamp near-zero noise to exactly 0
    if (fabsf(filtered) < VEL_ZERO_THRESH) {
        filtered = 0.0f;
    }

    sensor->prev_vel     = vel_instant;
    sensor->filtered_vel = filtered;

    // Apply direction sign
    sensor->actual_vel = (sensor->sensor_dir == REVERSE_DIR) ? -filtered : filtered;
}
