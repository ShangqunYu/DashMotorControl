/*
 * angle_sensor.c
 */

#include "angle_sensor.h"
#include "FOC_math.h"
#include "hw_config.h"
#include "math_ops.h"
#include "user_config.h"
#include <string.h>   /* memcpy */
#include <math.h>     /* floorf, fabsf */
#include <stdio.h>

// void angle_sensor_init(AngleSensor_t *sensor,
//                        uint8_t        pole_pairs,
//                        float          e_zero_rad,
//                        dir_mode_t     sensor_dir)
// {
//     if (sensor == NULL) return;

//     sensor->pole_pairs       = pole_pairs;
//     sensor->e_zero           = e_zero_rad;
//     sensor->m_zero           = 0.0f;
//     sensor->sensor_dir       = sensor_dir;
//     sensor->s_rotor_rad      = 0.0f;
//     sensor->s_rotor_rad_raw  = 0.0f;
//     sensor->e_rad            = 0.0f;
//     sensor->rotor_vel  = 0.0f;
//     sensor->pos_hat    = 0.0f;
//     sensor->vel_hat    = 0.0f;
//     sensor->obs_ready  = 0U;
//     sensor->lut_ready        = 0U;
//     sensor->turns            = 0;
//     sensor->first_sample     = 0;
//     sensor->multi_rotor_rad  = 0.0f;
// }

AngleSensor_t angle_sensor_init() {
    AngleSensor_t angle_sensor;
	angle_sensor.e_zero = E_ZERO_RAD;
    angle_sensor.m_zero = M_ZERO_RAD;
	angle_sensor.sensor_dir = PHASE_ORDER;
    angle_sensor.pole_pairs = (uint8_t)PPAIRS;
    if (CALIBRATION_DONE_FLAG == 1) {
        memcpy(angle_sensor.encd_error_comp, &ENCODER_LUT, sizeof(angle_sensor.encd_error_comp));
        angle_sensor.lut_ready = 1;
        printf("Encoder cal loaded: e_zero=%.4f, ppairs=%d, dir=%s\r\n",
            angle_sensor.e_zero, angle_sensor.pole_pairs,
            (angle_sensor.sensor_dir == REVERSE_DIR) ? "rev" : "norm");
    }
    angle_sensor.mech_angle_rad = 0;
    angle_sensor.mech_angle_vel = 0;
    angle_sensor.e_rad          = 0;
    angle_sensor.first_sample   = 0;
    angle_sensor.lut_ready      = 0;
    angle_sensor.pos_hat        = 0;
    angle_sensor.rotor_vel      = 0;
    angle_sensor.turns          = 0;
    angle_sensor.multi_rotor_rad= 0;
    angle_sensor.s_rotor_rad    = 0;
    angle_sensor.s_rotor_rad_raw= 0;
    angle_sensor.vel_hat        = 0;
    angle_sensor.obs_ready      = 0;


    return angle_sensor;
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

    float raw_rad = MA732_get_rad(&sensor->ma732);
    float old_s_angle = sensor->s_rotor_rad;

    // Angle referenced to electrical zero — used for LUT lookup and e_angle
    float angle_from_ezero = raw_rad - sensor->e_zero;
    norm_angle_rad(&angle_from_ezero);

    // Raw user position (before LUT correction, for comparison in ENCODER_MODE)
    // sensor->s_rotor_rad_raw = angle_from_ezero - sensor->m_zero;
    // norm_angle_rad(&sensor->s_rotor_rad_raw);

    // Apply LUT nonlinearity correction (indexed by angle from e_zero)
    if (sensor->lut_ready) {
        float lut_idx_f  = angle_from_ezero * ((float)ERROR_LUT_SIZE / TWO_PI);
        int   idx0       = (int)lut_idx_f;
        int   idx1       = idx0 + 1 < ERROR_LUT_SIZE ? idx0 + 1 : 0;  // no integer division
        float frac       = lut_idx_f - (float)idx0;
        float correction = sensor->encd_error_comp[idx0] * (1.0f - frac)
                         + sensor->encd_error_comp[idx1] * frac;
        angle_from_ezero -= correction;
        if      (angle_from_ezero < 0.0f)    angle_from_ezero += TWO_PI;
        else if (angle_from_ezero >= TWO_PI) angle_from_ezero -= TWO_PI;
    }

    // User-facing position: corrected angle minus user-defined mechanical zero
    sensor->s_rotor_rad = angle_from_ezero - sensor->m_zero;
    if      (sensor->s_rotor_rad < 0.0f)     sensor->s_rotor_rad += TWO_PI;
    else if (sensor->s_rotor_rad  >= TWO_PI) sensor->s_rotor_rad -= TWO_PI;

    // Multi-turn tracking — skip rollover on first sample to avoid a false jump
    // from uninitialized old_s_angle
    float diff = sensor->s_rotor_rad - old_s_angle;
    if      (diff >  PI) sensor->turns--;
    else if (diff < -PI) sensor->turns++;

    if (!sensor->first_sample) {
        sensor->first_sample = 1;
        sensor->turns = (sensor->s_rotor_rad > PI) ? -1 : 0;
    }
    sensor->multi_rotor_rad = sensor->s_rotor_rad + TWO_PI * (float)sensor->turns;
    if (sensor->sensor_dir == REVERSE_DIR)
        sensor->multi_rotor_rad = -sensor->multi_rotor_rad;

    sensor->mech_angle_rad = sensor->multi_rotor_rad * (1.0f / GR);

    // Electrical angle (from e_zero, not m_zero — FOC must stay anchored to e_zero)
    float e_rad = angle_from_ezero * (float)sensor->pole_pairs;
    if (sensor->sensor_dir == REVERSE_DIR) {
        e_rad = TWO_PI - e_rad;
    }
    norm_angle_rad(&e_rad);
    sensor->e_rad = e_rad;

    MA732_set_val_flag();
}

void angle_sensor_set_m_zero(AngleSensor_t *sensor)
{
    /*
     * To make the current position to be the new mechanical zero
     * We know that s_rotor_rad = angle_from_ezero - m_zero
     * m_zero := m_zero + s_rotor_rad = m_zero + angle_from_ezero - m_zero = angle_from_ezero
    */
    sensor->m_zero += sensor->s_rotor_rad;
    norm_angle_rad(&sensor->m_zero);
    sensor->s_rotor_rad      = 0.0f;
    sensor->s_rotor_rad_raw  = 0.0f;
    sensor->turns            = 0;
    sensor->multi_rotor_rad  = 0.0f;
    sensor->mech_angle_rad   = 0.0f;
}

void angle_sensor_update_velocity(AngleSensor_t *sensor)
{
    if (!sensor->obs_ready) {
        sensor->pos_hat   = sensor->multi_rotor_rad;
        sensor->vel_hat   = 0.0f;
        sensor->obs_ready = 1U;
    }

    // Predict
    sensor->pos_hat += FOC_TS * sensor->vel_hat;

    // Correct
    float error = sensor->multi_rotor_rad - sensor->pos_hat;
    sensor->pos_hat += OBS_L1 * error;
    sensor->vel_hat += OBS_L2 * error;

    float vel = sensor->vel_hat;
    if (fabsf(vel) < VEL_ZERO_THRESH)
        vel = 0.0f;

    sensor->rotor_vel = vel;
    sensor->mech_angle_vel = vel / GR;
}
