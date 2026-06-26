/*
 * analog_sensor.h
 *
 * ADC-based sensor drivers: current sensing (voltage and temperature to follow).
 */

#ifndef INC_ANALOG_SENSOR_H_
#define INC_ANALOG_SENSOR_H_

#include <stdint.h>

typedef struct {
    volatile uint32_t *adc_ia;
    volatile uint32_t *adc_ib;
    volatile uint32_t *adc_ic;
    float current_scale;
    int adc_a_offset;
    int adc_b_offset;
    int adc_c_offset;
    float ia;
    float ib;
    float ic;
    float ia_filtered;
    float ib_filtered;
    float ic_filtered;
    volatile uint8_t offset_calibrating;
    volatile uint32_t adc_a_offset_sum;
    volatile uint32_t adc_b_offset_sum;
    volatile uint32_t adc_c_offset_sum;
    volatile uint32_t adc_offset_sample_count;
} CurrentSensor;

void CurrentSensor_init(CurrentSensor *sensor,
                        volatile uint32_t *adc_ia,
                        volatile uint32_t *adc_ib,
                        volatile uint32_t *adc_ic,
                        float current_scale,
                        int adc_a_offset,
                        int adc_b_offset,
                        int adc_c_offset);
void CurrentSensor_update(CurrentSensor *sensor);
void CurrentSensor_begin_offset_calibration(CurrentSensor *sensor);
void CurrentSensor_sample_offset(CurrentSensor *sensor);
void CurrentSensor_end_offset_calibration(CurrentSensor *sensor);
void CurrentSensor_calibrate(CurrentSensor *sensor, uint32_t duration_ms);

void update_power_voltage(float *v_bus);
void update_temperature(float *motor_temp);

#endif /* INC_ANALOG_SENSOR_H_ */
