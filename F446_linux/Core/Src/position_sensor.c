/*
 * position_sensor.c
 *
 *  Created on: Jan 12, 2026
 *      Author: simon
 */

#include "position_sensor.h"
#include "hw_config.h"

uint16_t readMagAlphaAngle(void)
{
  uint32_t timeout=10;
  uint8_t txData[2];
  uint8_t rxData[2];
  txData[1]=0;
  txData[0]=0;
  uint16_t angleSensor;
  HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET );
  HAL_SPI_TransmitReceive(&ENC_SPI, txData, rxData, 2, timeout);
  HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
  angleSensor=rxData[0] | rxData[1] <<8;
  return angleSensor;
}
