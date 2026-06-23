/*
 * MA732.C
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#include "MA732.h"
#include <string.h>
#include <math.h>
#include "math_ops.h"

volatile _Bool encd_get_val_flag = 0;


int MA732_config(MA732_t *encd, SPI_HandleTypeDef *hspi) {
    if (encd == NULL || hspi == NULL) {
        return 0;
    }

    encd->MA732_spi = hspi;
    
    MA732_cs_set();
    
    return 1;
}


int MA732_start(MA732_t *encd) {
    uint16_t cmd = ENC_READ_WORD;

	MA732_cs_reset();
	if (HAL_SPI_TransmitReceive_DMA(encd->MA732_spi, (uint8_t*)&cmd, encd->spi_rx_buffer, 1) != HAL_OK) {
        return 0;
    }

	return 1;
}


float MA732_get_rad(MA732_t *encd) {
    MA732_cs_set();

    const uint16_t raw_data = ((uint16_t)encd->spi_rx_buffer[1] << 8) | encd->spi_rx_buffer[0];

    encd->angle_raw = (float)(raw_data >> 2) * ANGLE_SCALE_FACTOR;
    return encd->angle_raw;
}
