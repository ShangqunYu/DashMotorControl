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



#define ANGLE_SCALE_FACTOR   0.00038349519824f  // 2π / 16384 (14-bit)



typedef struct {
    SPI_HandleTypeDef *MA732_spi;
    uint8_t spi_rx_buffer[2];
    float angle_raw;
}MA732_t;

#define MA732_cs_set() (HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET))
#define MA732_cs_reset() (HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET))

#define MA732_get_val_flag() (encd_get_val_flag == 1)
#define MA732_set_val_flag() (encd_get_val_flag = 1)
#define MA732_reset_val_flag() (encd_get_val_flag = 0)


extern volatile _Bool encd_get_val_flag;

int MA732_config(MA732_t *encd, SPI_HandleTypeDef *hspi);
int MA732_start(MA732_t *encd);
float MA732_get_rad(MA732_t *encd);

#endif /* MA732_DRIVER_INC_MA732_H_ */
