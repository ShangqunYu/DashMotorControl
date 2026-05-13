/*
 * MA732.C
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#include "MA732.h"
#include <string.h>
#include <math.h>

#define MA732_REG_ANGLE  0x3FFF
#define MA732_WRITE_CMD  0x4000

_Bool encd_get_val_flag = 0;


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


float MA732_get_degree(MA732_t *encd) {
    MA732_cs_set();

    const uint16_t raw_data = ((uint16_t)encd->spi_rx_buffer[1] << 8) | encd->spi_rx_buffer[0];


#ifdef HIGH_RES
    const float angle_raw = (float)(raw_data & 0xFFF0) * ANGLE_SCALE_FACTOR;
#else
    const float angle_raw = (float)((raw_data >> 2) & 0x0FFF) * ANGLE_SCALE_FACTOR;
#endif

    float angle_diff = angle_raw - encd->prev_raw_angle;
    angle_diff -= 360.0f * floorf((angle_diff + 180.0f) / 360.0f);

    if (fabsf(angle_diff) > MAX_ANGLE_JUMP_DEG) {
        if (++encd->spike_counter < SPIKE_REJECT_COUNT) {
            return encd->angle_filtered;
        }
        encd->spike_counter = 0;
    } else {
        encd->spike_counter = 0;
    }

    encd->prev_raw_angle = angle_raw;

    // Filter IIR dengan wrap-around
    float filtered_diff = angle_raw - encd->angle_filtered;
    filtered_diff -= 360.0f * floorf((filtered_diff + 180.0f) / 360.0f);
    encd->angle_filtered += ANGLE_FILTER_ALPHA * filtered_diff;

    if (encd->angle_filtered >= 360.0f)
        encd->angle_filtered -= 360.0f;
    else if (encd->angle_filtered < 0.0f)
        encd->angle_filtered += 360.0f;

    return encd->angle_filtered;
}
