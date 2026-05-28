/*
 * analog_sensor.c
 *
 * ADC-based sensor drivers: current sensing (voltage and temperature to follow).
 */

#include "analog_sensor.h"
#include "hw_config.h"
#include "stm32f4xx_hal.h"

void CurrentSensor_init(CurrentSensor *sensor,
                        volatile uint32_t *adc_ia,
                        volatile uint32_t *adc_ib,
                        float current_scale,
                        int adc_a_offset,
                        int adc_b_offset) {
    sensor->adc_ia = adc_ia;
    sensor->adc_ib = adc_ib;
    sensor->current_scale = current_scale;
    sensor->adc_a_offset = adc_a_offset;
    sensor->adc_b_offset = adc_b_offset;
    sensor->ia = 0.0f;
    sensor->ib = 0.0f;
    sensor->ic = 0.0f;
    sensor->ia_filtered = 0.0f;
    sensor->ib_filtered = 0.0f;
    sensor->ic_filtered = 0.0f;
    sensor->offset_calibrating = 0U;
    sensor->adc_a_offset_sum = 0U;
    sensor->adc_b_offset_sum = 0U;
    sensor->adc_offset_sample_count = 0U;
}

void CurrentSensor_update(CurrentSensor *sensor) {
    sensor->ia = -(sensor->adc_a_offset - (float)(*sensor->adc_ia)) * sensor->current_scale;
    sensor->ib = -(sensor->adc_b_offset - (float)(*sensor->adc_ib)) * sensor->current_scale;
    sensor->ic = -sensor->ia - sensor->ib;
    sensor->ia_filtered = (1.0f - CURRENT_FILTER_ALPHA) * sensor->ia_filtered + CURRENT_FILTER_ALPHA * sensor->ia;
    sensor->ib_filtered = (1.0f - CURRENT_FILTER_ALPHA) * sensor->ib_filtered + CURRENT_FILTER_ALPHA * sensor->ib;
    sensor->ic_filtered = -sensor->ia_filtered - sensor->ib_filtered;
}

void CurrentSensor_begin_offset_calibration(CurrentSensor *sensor) {
    sensor->offset_calibrating = 1U;
    sensor->adc_a_offset_sum = 0U;
    sensor->adc_b_offset_sum = 0U;
    sensor->adc_offset_sample_count = 0U;
}

void CurrentSensor_sample_offset(CurrentSensor *sensor) {
    if (sensor->offset_calibrating == 0U) return;
    sensor->adc_a_offset_sum += *sensor->adc_ia;
    sensor->adc_b_offset_sum += *sensor->adc_ib;
    sensor->adc_offset_sample_count++;
}

void CurrentSensor_end_offset_calibration(CurrentSensor *sensor) {
    sensor->offset_calibrating = 0U;
    if (sensor->adc_offset_sample_count > 0U) {
        sensor->adc_a_offset = (int)(sensor->adc_a_offset_sum / sensor->adc_offset_sample_count);
        sensor->adc_b_offset = (int)(sensor->adc_b_offset_sum / sensor->adc_offset_sample_count);
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

float get_power_voltage(void) {
    static float pv_filtered = 0.0f;
    const float filter_alpha = 0.2f;
    float pv = (float)ADC3->JDR1 * V_SCALE;
    pv_filtered = (1.0f - filter_alpha) * pv_filtered + filter_alpha * pv;
    return pv_filtered;
}
