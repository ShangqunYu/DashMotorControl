/*
 * angle_sensor.c
 */

#include "angle_sensor.h"
#include "math_utils.h"
#include "hw_config.h"
#include "user_config.h"
#include <stdint.h>
#include <string.h>   /* memcpy */
#include <math.h>     /* floorf, fabsf */
#include <stdio.h>
#include "spi.h"

static GPIO_TypeDef *enc_cs_port;
static uint16_t      enc_cs_pin;



float encoder_get_rad(AngleSensor_t *sensor) {
    HAL_GPIO_WritePin(enc_cs_port, enc_cs_pin, GPIO_PIN_SET);
    const uint16_t raw_data = sensor->spi_rx_buffer;
    float angle_scale_factor = 0.00038349519824f;   // 2π / 16384 (14-bit)
    sensor->raw_rad = (float)(raw_data >> 2) * angle_scale_factor;
    return sensor->raw_rad;
}

int encoder_start(AngleSensor_t *sensor) {
    uint16_t cmd = ENC_READ_WORD;

	HAL_GPIO_WritePin(enc_cs_port, enc_cs_pin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(&ENC_SPI, (uint8_t*)&cmd, (uint8_t*)&sensor->spi_rx_buffer, 1) != HAL_OK) {
        return 0;
    }

	return 1;
}




AngleSensor_t angle_sensor_init() {
    AngleSensor_t angle_sensor;
	angle_sensor.e_zero          = CFG_E_ZERO_RAD;
    angle_sensor.m_zero          = CFG_M_ZERO_RAD;
	angle_sensor.sensor_dir      = CFG_PHASE_ORDER;
    angle_sensor.pole_pairs      = (uint8_t)CFG_PPAIRS;
    angle_sensor.mech_angle_rad  = 0;
    angle_sensor.mech_angle_vel  = 0;
    angle_sensor.e_rad           = 0;
    angle_sensor.first_sample    = 0;
    angle_sensor.lut_ready       = 0;
    if (CFG_CALIBRATION_DONE_FLAG == 1) {
        angle_sensor_load_lut(&angle_sensor, (const float *)&CFG_ENCODER_LUT, ERROR_LUT_SIZE);
        printf("Encoder cal loaded: e_zero=%.4f, ppairs=%d, dir=%s\r\n",
            angle_sensor.e_zero, angle_sensor.pole_pairs,
            (angle_sensor.sensor_dir == REVERSE_DIR) ? "rev" : "norm");
    }
    angle_sensor.pos_hat        = 0;
    angle_sensor.rotor_vel      = 0;
    angle_sensor.turns          = 0;
    angle_sensor.multi_rotor_rad= 0;
    angle_sensor.single_rotor_rad    = 0;
    angle_sensor.single_rotor_rad_raw= 0;
    angle_sensor.vel_hat        = 0;
    angle_sensor.obs_ready      = 0;
    angle_sensor.raw_rad          = 0;
    angle_sensor.encd_get_val_flag = 1;  /* prime the DMA pump so the first FOC cycle kicks a transfer */

    if (CFG_ENC_SEL == 0) {
        enc_cs_port = ENC_CS_INT_PORT;
        enc_cs_pin  = ENC_CS_INT_PIN;
    } else {
        enc_cs_port = ENC_CS_EXT_PORT;
        enc_cs_pin  = ENC_CS_EXT_PIN;
        HAL_GPIO_WritePin(ENC_CS_INT, GPIO_PIN_SET);  /* deassert internal CS */
    }
    HAL_GPIO_WritePin(enc_cs_port, enc_cs_pin, GPIO_PIN_SET);

      /* MA732 setup */
    for (int i=0; i<20; i++) {
        // MA732_get_rad();
        encoder_start(&angle_sensor);
        HAL_Delay(10);
    }
    return angle_sensor;
}

void angle_sensor_load_lut(AngleSensor_t *sensor,
                           const float   *lut,
                           uint16_t       lut_size)
{
    uint16_t copy_size = lut_size < ERROR_LUT_SIZE ? lut_size : ERROR_LUT_SIZE;
    memcpy(sensor->encd_error_comp, lut, copy_size * sizeof(float));
    sensor->lut_ready = 1U;
}

void angle_sensor_update(AngleSensor_t *sensor)
{
    angle_sensor_update_position(sensor);
    angle_sensor_update_velocity(sensor);
}

void angle_sensor_set_m_zero(AngleSensor_t *sensor)
{
    /*
     * To make the current position to be the new mechanical zero
     * We know that s_rotor_rad = angle_from_ezero - m_zero
     * m_zero := m_zero + s_rotor_rad = m_zero + angle_from_ezero - m_zero = angle_from_ezero
    */
    sensor->m_zero += sensor->single_rotor_rad;
    norm_angle_rad(&sensor->m_zero);
    sensor->single_rotor_rad      = 0.0f;
    sensor->single_rotor_rad_raw  = 0.0f;
    sensor->turns            = 0;
    sensor->multi_rotor_rad  = 0.0f;
    sensor->mech_angle_rad   = 0.0f;
}

void angle_sensor_update_position(AngleSensor_t *sensor) {
    // sensor->raw_rad = MA732_get_rad();
    if (sensor->encd_get_val_flag) {
        encoder_start(sensor);
        sensor->encd_get_val_flag = 0;
    }
    float old_s_angle = sensor->single_rotor_rad;

    // Angle referenced to electrical zero — used for LUT lookup and e_angle
    float angle_from_ezero = sensor->raw_rad - sensor->e_zero;
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
    sensor->single_rotor_rad = angle_from_ezero - sensor->m_zero;
    if      (sensor->single_rotor_rad < 0.0f)     sensor->single_rotor_rad += TWO_PI;
    else if (sensor->single_rotor_rad  >= TWO_PI) sensor->single_rotor_rad -= TWO_PI;

    // Multi-turn tracking — skip rollover on first sample to avoid a false jump
    // from uninitialized old_s_angle
    float diff = sensor->single_rotor_rad - old_s_angle;
    if      (diff >  PI) sensor->turns--;
    else if (diff < -PI) sensor->turns++;

    if (!sensor->first_sample) {
        sensor->first_sample = 1;
        sensor->turns = (sensor->single_rotor_rad > PI) ? -1 : 0;
    }
    sensor->multi_rotor_rad = sensor->single_rotor_rad + TWO_PI * (float)sensor->turns;
    if (sensor->sensor_dir == REVERSE_DIR)
        sensor->multi_rotor_rad = -sensor->multi_rotor_rad;

    sensor->mech_angle_rad = sensor->multi_rotor_rad * (1.0f / CFG_GR);

    // Electrical angle (from e_zero, not m_zero — FOC must stay anchored to e_zero)
    float e_rad = angle_from_ezero * (float)sensor->pole_pairs;
    if (sensor->sensor_dir == REVERSE_DIR) {
        e_rad = TWO_PI - e_rad;
    }
    // the electric angle in radiance is not normalized, it will be normalized in the fast_sincos function in math_utils.c
    sensor->e_rad = e_rad;
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
    sensor->mech_angle_vel = vel / CFG_GR;
}


// float MA732_get_rad(){
//     HAL_GPIO_WritePin(enc_cs_port, enc_cs_pin, GPIO_PIN_RESET);
//     uint8_t spi_tx_buffer[] = {0,0};
//     uint8_t spi_rx_buffer[2];
//     HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)spi_tx_buffer, (uint8_t *)spi_rx_buffer, 1, 1);
//     HAL_GPIO_WritePin(enc_cs_port, enc_cs_pin, GPIO_PIN_SET);
//     const uint16_t raw_data = ((uint16_t)spi_rx_buffer[1] << 8) | spi_rx_buffer[0];
//     float angle_scale_factor = 0.00038349519824f;   // 2π / 16384 (14-bit)
//     float angle_raw = (float)(raw_data >> 2) * angle_scale_factor;
//     return angle_raw;
// }



// // "Member Variables"
// // https://stackoverflow.com/questions/7259830/why-and-when-to-use-static-structures-in-c-programming
// static uint16_t rawBitsFromSPI;
// static float rotorTurnsPerElectricalCycle;
// static float electricalCyclesPerRotorTurn;
// static float rotorTurnsWhenElectricalAngleIsZero;
// static int rotorFullTurnCount;


// static const float radiansPerTurn = 2 * M_PI;
// static const float turnsPerRadian = 1.0 / radiansPerTurn;
// static const float degreesPerTurn = 360;
// static const float turnsPerDegree = 1.0 / degreesPerTurn;



// void angle_sensor_init_TEST() {
//     rawBitsFromSPI = 0;
//     rotorTurnsPerElectricalCycle = 1;
//     electricalCyclesPerRotorTurn = 1;
//     rotorTurnsWhenElectricalAngleIsZero = 0;
//     rotorFullTurnCount = 0;
// }

// // Gives angles restricted to the range [0, 1]
// float get_absolute_rotor_angle_turns() {
//     return 0;
// }
// float get_absolute_rotor_angle_radians() {
//     return radiansPerTurn * get_absolute_rotor_angle_turns();
// }
// float get_absolute_rotor_angle_degrees() {
//     return degreesPerTurn * get_absolute_rotor_angle_turns();
// }

// float rotor_turns_to_electrical_turns(float rotorTurns) {

// }

// float get_absolute_electrical_angle_turns() {
//     // Before scaling the rotor angle by [electricalCyclesPerRotorTurn],
//     // we must account for the fact that rotorZero and electricalZero
//     // aren't guaranteed to line up with one another.
//     // TODO: Add details explaining how the rotor angle coordinate system depends on arbitrary encoder magnet installation,
//     //       whereas the electrical angle coordinate system depends on how the rotor magnets are positioned relative to the stator coils.
//     //       Be sure to emphasize that "rotor magnets" aren't the same as "encoder magnet" despite both being on the rotor.
//     //       "encoder manget" is used for measuring the rotor angle, "rotor magnets" are used for pushing on / spinning the rotor.
//     float rotorTurns_awayFromElectricalZero = get_absolute_rotor_angle_turns() - rotorTurnsWhenElectricalAngleIsZero; // This is the displacement from "electrical zero", measured in units of "rotor turns"
//     float electricalTurns_awayFromElectricalZero = rotorTurns_awayFromElectricalZero * electricalCyclesPerRotorTurn;                                  // Now take that displacement, and convert it into units of "electrical turns"

//     // Between the shift and the scaling, the value of [scaled] has likely been pushed outside
//     // of the range [0, 1], so we wrap it around to the equivalent angle in that range.
//     return circular_clamp(electricalTurns_awayFromElectricalZero, 0, 1);
// }

// float get_absolute_electrical_angle_radians() {
//     return radiansPerTurn * get_absolute_electrical_angle_turns();
// }

// float get_absolute_electrical_angle_degrees() {
//     return degreesPerTurn * get_absolute_electrical_angle_turns();
// }



// // Intended for wrapping angles to a restricted range, while preserving the "direction" the angle points in.
// // TODO: Better docs?
// static float circular_clamp(float wrapMe, float inclusiveMin, float exclusiveMax) {
//     // The easiest way to implement this function (in terms of understanding) is probably:
//     //   float distanceBetweenEquivalentValues =  exclusiveMax - inclusiveMin;
//     //   while (wrapMe <  min) { wrapMe += distanceBetweenEquivalentValues; }
//     //   while (wrapMe >= max) { wrapMe -= distanceBetweenEquivalentValues; }
//     //   return wrapMe;
//     //
//     // However, you hopefully get the feeling that there's a way to just calculate the answer
//     // directly instead of looping until you're in the desired range, which is what we do below.

//     // 1) Find the number that we're allowed to add or subtract from "wrapMe" without
//     //    changing its effective value.
//     //    (e.g. when wrapping angles measured in degrees, this value is 360. Adding or subtracting
//     //     360 to any given angle results in an angle pointing in the same direction as the original.)
//     float distanceBetweenEquivalentValues = exclusiveMax - inclusiveMin;

//     // 2) Find the "t" value for if "wrapMe" had been obtained by
//     //    linearly interpolating between the min and the max.
//     float t = (wrapMe - inclusiveMin) / distanceBetweenEquivalentValues;

//     // 3) Use the non-fractional part of "t" (i.e. the part that's before the decimal place) to calculate the "index"
//     //    of the current range. For example, if we're clamping to the range 0 to 360, then:
//     //
//     //      [0-360) has index 0
//     //      [360-720) has index 1
//     //      [720-1080) has index 2
//     //      [-360-0) has index -1
//     //      [-720-360) has index -2, etc.
//     float indexOfCurrentRange = floorf(t);
//     // Note: You need to use Math.floor() here as opposed to just casting to int. This is because casting
//     //       to int always rounds towards 0, whereas the floor of a negative number rounds away from 0.
//     //       See: https://www.desmos.com/calculator/pnmjpb1whn

//     // 4) Get the final result by subtracting the appropriate amount of
//     //    full ranges from the given number.
//     return wrapMe - (distanceBetweenEquivalentValues * indexOfCurrentRange);
// }


