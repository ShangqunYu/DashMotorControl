/*
 * MA732.h
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */



#ifndef MA732_DRIVER_INC_MA732_H_
#define MA732_DRIVER_INC_MA732_H_

#include "hw_config.h"
#include "stm32f4xx_hal.h"

#define HIGH_RES

#define ANGLE_FILTER_ALPHA    0.71539f  //0.71539f //   // Faktor filter
#define MAX_ANGLE_JUMP_DEG    50.0f   // Batas maksimal lonjakan sudut (derajat)
#define SPIKE_REJECT_COUNT    10       // Jumlah sampel untuk konfirmasi spike

// Configurations (tune these based on your system)
#define MIN_DT_US             500UL      // Minimum 1ms interval for valid RPM (avoid division by tiny numbers)
#define MAX_RPM_JUMP          50.0f      // Conservative RPM jump threshold
#define RPM_FILTER_ALPHA      0.71539f //0.71539f      // Base filter coefficient (balanced response)
#define DEGREES_PER_REV       360.0f     // For 1:1 gear ratio
#define MICROS_TO_MINUTES     6e7f       // Conversion factor (μs to minutes)

#define GEAR_RATIO 1.0f //0.0526315789f
#define ACTUAL_ANGLE_OFFSET 60.0f
// #define ACTUAL_ANGLE_OFFSET (-75.0f)
#define ACTUAL_ANGLE_FILTER_ALPHA 0.71539f


#ifdef HIGH_RES
// #define ANGLE_SCALE_FACTOR    0.021973997F  // Pre-calculate scale factor (360.0f / 16383.0f)
#define ANGLE_SCALE_FACTOR   0.00549324788f // re-calculate scale factor (360.0f / 65535.0f)
#else
#define ANGLE_SCALE_FACTOR    (360.0f / 4095.0f) 
#endif


typedef struct {
    SPI_HandleTypeDef *MA732_spi;
    uint8_t spi_rx_buffer[2];
    
    float angle_filtered;
    float prev_angle_filtered;
    float prev_raw_angle;
    uint8_t spike_counter;
    
    float prev_angle;
    float filtered_rpm;
    float prev_rpm;
    float angle_accumulator;
    uint32_t time_accumulator;
    
	float output_prev_angle;
	float output_angle_ovf;
    float output_angle_filtered;
}MA732_t;

#define MA732_cs_set() (HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET))
#define MA732_cs_reset() (HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET))

#define MA732_get_val_flag() (encd_get_val_flag == 1)
#define MA732_set_val_flag() (encd_get_val_flag = 1)
#define MA732_reset_val_flag() (encd_get_val_flag = 0)


extern _Bool encd_get_val_flag;

int MA732_config(MA732_t *encd, SPI_HandleTypeDef *hspi);
int MA732_start(MA732_t *encd);
float MA732_get_degree(MA732_t *encd);
float MA732_get_rpm(MA732_t *encd, float Ts);

#endif /* MA732_DRIVER_INC_MA732_H_ */
