/*
 * stm32_bitbang_spi.h
 *
 *  Created on: Nov 5, 2025
 *      Author: yusha
 */

#ifndef INC_STM32_BITBANG_SPI_H_
#define INC_STM32_BITBANG_SPI_H_

#include "main.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>

/**
 * @brief Options for bit-banged SPI configuration
 */
typedef struct {
    GPIO_TypeDef* mosi_port;
    uint16_t      mosi_pin;
    GPIO_TypeDef* miso_port;
    uint16_t      miso_pin;
    GPIO_TypeDef* sck_port;
    uint16_t      sck_pin;
    GPIO_TypeDef* cs_port;
    uint16_t      cs_pin;
    int           frequency;   // default 1 MHz
    int           width;       // number of bits per transfer
    int           mode;        // CPOL=0, CPHA=1 (moteus compatible)
} BitbangSPI_Options;

/**
 * @brief Bitbang SPI context (runtime object)
 */
typedef struct {
    BitbangSPI_Options options;
    uint32_t us_delay;
} BitbangSPI;

/**
 * @brief Initialize bit-banged SPI interface
 */
void BitbangSPI_Init(BitbangSPI* spi, const BitbangSPI_Options* options);

/**
 * @brief Write one word over bit-banged SPI and read back response
 */
uint16_t BitbangSPI_Write(BitbangSPI* spi, uint16_t value);

uint16_t BitbangSPI_Read(BitbangSPI* spi, uint16_t addr);

#endif /* INC_STM32_BITBANG_SPI_H_ */
