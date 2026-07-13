/*
 * analog_sensor.c
 *
 * ADC-based sensor drivers: current sensing (voltage and temperature to follow).
 */

#include "analog_sensor.h"
#include "hw_config.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

void CurrentSensor_init(CurrentSensor *sensor,
                        volatile uint32_t *adc_ia,
                        volatile uint32_t *adc_ib,
                        volatile uint32_t *adc_ic,
                        float current_scale,
                        int adc_a_offset,
                        int adc_b_offset,
                        int adc_c_offset) {
    sensor->adc_ia = adc_ia;
    sensor->adc_ib = adc_ib;
    sensor->adc_ic = adc_ic;
    sensor->current_scale = current_scale;
    sensor->adc_a_offset = adc_a_offset;
    sensor->adc_b_offset = adc_b_offset;
    sensor->adc_c_offset = adc_c_offset;
    sensor->ia = 0.0f;
    sensor->ib = 0.0f;
    sensor->ic = 0.0f;
    sensor->offset_calibrating = 0U;
    sensor->adc_a_offset_sum = 0U;
    sensor->adc_b_offset_sum = 0U;
    sensor->adc_c_offset_sum = 0U;
    sensor->adc_offset_sample_count = 0U;
}

void CurrentSensor_update(CurrentSensor *sensor) {
    sensor->ia = (sensor->adc_a_offset - (float)(*sensor->adc_ia)) * sensor->current_scale;
    sensor->ib = (sensor->adc_b_offset - (float)(*sensor->adc_ib)) * sensor->current_scale;
    sensor->ic = (sensor->adc_c_offset - (float)(*sensor->adc_ic)) * sensor->current_scale;
}

void CurrentSensor_begin_offset_calibration(CurrentSensor *sensor) {
    sensor->offset_calibrating = 1U;
    sensor->adc_a_offset_sum = 0U;
    sensor->adc_b_offset_sum = 0U;
    sensor->adc_c_offset_sum = 0U;
    sensor->adc_offset_sample_count = 0U;
}

void CurrentSensor_sample_offset(CurrentSensor *sensor) {
    if (sensor->offset_calibrating == 0U) return;
    sensor->adc_a_offset_sum += *sensor->adc_ia;
    sensor->adc_b_offset_sum += *sensor->adc_ib;
    sensor->adc_c_offset_sum += *sensor->adc_ic;
    sensor->adc_offset_sample_count++;
}

void CurrentSensor_end_offset_calibration(CurrentSensor *sensor) {
    sensor->offset_calibrating = 0U;
    if (sensor->adc_offset_sample_count > 0U) {
        sensor->adc_a_offset = (int)(sensor->adc_a_offset_sum / sensor->adc_offset_sample_count);
        sensor->adc_b_offset = (int)(sensor->adc_b_offset_sum / sensor->adc_offset_sample_count);
        sensor->adc_c_offset = (int)(sensor->adc_c_offset_sum / sensor->adc_offset_sample_count);
    }
}

void CurrentSensor_calibrate(CurrentSensor *sensor, uint32_t duration_ms) {
    uint32_t start_tick = HAL_GetTick();
    CurrentSensor_begin_offset_calibration(sensor);
    while ((HAL_GetTick() - start_tick) < duration_ms) {
        HAL_Delay(1);
    }
    CurrentSensor_end_offset_calibration(sensor);
}

void update_power_voltage(float *v_bus) {
    float raw = (float)ADC2->JDR2 * V_SCALE;
    *v_bus += VBUS_FILT_ALPHA * (raw - *v_bus);
}

void update_temperature(float *motor_temp) {
    // Voltage divider: VCC →  thermistor → ADC_pin → BASE_RESISTOR → GND
    // Adc reading is measuring the base resistor voltage drop. 
    float adc_raw = (float)ADC3->JDR2;
    float thermistor_resistance = BASE_RESISTOR_RESISTANCE * (4095.0f - adc_raw) / adc_raw;
    float temp    = THERMISTOR_NOMINAL_TEMP + (thermistor_resistance - THERMISTOR_NOMINAL_RESISTANCE) / OHM_PER_DEGREE_C;
    *motor_temp  += TEMP_FILT_ALPHA * (temp - *motor_temp);
}