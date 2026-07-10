/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define T_MIN -18.0f
#define T_MAX 18.0f
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
}CANRxMessage ;

typedef struct{
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

/* Flash-signal flags, set by the CAN RX ISR on a param write and consumed by the
 * flash-writing task. pending_reboot marks params only consumed at init (encoder
 * select), so the task reboots after the write is committed. Defined in can.c. */
extern volatile uint8_t pending_save;
extern volatile uint8_t pending_reboot;

void init_can_rx_filter();
// void can_tx_init();
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb, float temp);
void unpack_cmd(CANRxMessage msg, float *commands);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

