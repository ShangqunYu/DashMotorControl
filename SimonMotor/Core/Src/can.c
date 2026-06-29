/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>
#include "hw_config.h"
#include "user_config.h"
#include "foc.h"
#include "fsm.h"
#include <stdio.h>
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
extern foc_t              hfoc;
extern FSMStruct          hfsm;


void init_can_rx_filter(){
  CAN_FilterTypeDef filter;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterBank = 10;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterIdHigh = CAN_ID >> 13 & 0xFFFF;
  filter.FilterIdLow  = CAN_ID << 3 & 0xFFF8;
  filter.FilterMaskIdHigh = FILTER_MASK >> 13 & 0xFFFF;
  filter.FilterMaskIdLow = FILTER_MASK << 3 & 0xFFF8;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale=CAN_FILTERSCALE_32BIT;
  filter.SlaveStartFilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1, &filter);
}



// void can_tx_init(){
// 	message_to_send.tx_header.DLC = 8; 			// message size of 8 byte
// 	message_to_send.tx_header.IDE=CAN_ID_EXT; 		// set identifier to standard
// 	message_to_send.tx_header.RTR=CAN_RTR_DATA; 	// set data type to remote transmission request?
// 	message_to_send.tx_header.StdId = CAN_MASTER;  // recipient CAN ID
// }

void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb, float temp){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -I_MAX*KT*GR, I_MAX*KT*GR, 12);
    int vb_int = float_to_uint(vb, V_BUS_MIN, V_BUS_MAX, 8);
    int temp_int = float_to_uint(temp, TEMP_MIN, TEMP_MAX, 8);
    msg->data[0] = id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    msg->data[6] = vb_int;
    msg->data[7] = temp_int;
    }

void unpack_cmd(CANRxMessage msg, float *commands){// ControllerStruct * controller){
    int p_int = (msg.data[0]<<8)|msg.data[1];
    int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
    int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
    int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
    int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];

    commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
    commands[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
    commands[2] = uint_to_float(kp_int, 0, KP_MAX, 12);
    commands[3] = uint_to_float(kd_int, 0, KD_MAX, 12);
    commands[4] = uint_to_float(t_int, -I_MAX*KT*GR, I_MAX*KT*GR, 12);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    CANRxMessage received_message;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &received_message.rx_header, received_message.data);


    CANTxMessage message_to_send;
    message_to_send.tx_header.DLC = 8; 			// message size of 8 byte
    message_to_send.tx_header.IDE=CAN_ID_EXT; 		// set identifier to standard
    message_to_send.tx_header.RTR=CAN_RTR_DATA; 	// set data type to remote transmission request?
    message_to_send.tx_header.StdId = CAN_MASTER;  // recipient CAN ID
    /* Send reply: can_id, position (rad), velocity (rad/s), estimated torque (N-m), vbus (V), motor temp (C) */
    pack_reply(&message_to_send, CAN_ID, hfoc.angle_sensor.mech_angle_rad, hfoc.angle_sensor.mech_angle_vel, hfoc.iq*KT*GR, hfoc.v_bus, hfoc.motor_temp);
    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(&CAN_H, &message_to_send.tx_header, message_to_send.data, &tx_mailbox);


    /* Special commands: first 7 bytes all 0xFF, last byte selects command */
    bool first_seven_bytes_all_ones = true;
    for (int i = 0; i < 7; i += 1) {
        first_seven_bytes_all_ones =  first_seven_bytes_all_ones && (received_message.data[i] == 0xFF);
    }
    if (first_seven_bytes_all_ones) {
        switch (received_message.data[7]) {
            case MIT_MODE: update_fsm(&hfsm, MOTOR_CMD); break;  /* enter torque control */
            case MENU_MODE: update_fsm(&hfsm, MENU_CMD);  break;  /* return to menu / disable */
            case SET_ZERO_MODE: update_fsm(&hfsm, ZERO_CMD);  break;  /* set mechanical zero */
            case CALIBRATION_MODE: update_fsm(&hfsm, CAL_CMD); break;  /* enter calibration mode */
            case ENCODER_MODE: update_fsm(&hfsm, ENCODER_CMD); break;  /* enter encoder display mode */
            case R_MEAS_MODE: update_fsm(&hfsm, R_MEAS_CMD); break;  /* enter R measurement mode */
            case L_MEAS_MODE: update_fsm(&hfsm, L_MEAS_CMD); break;  /* enter L measurement mode */
            default:   break;
        }
        return;
    }

    /* Regular MIT position/velocity/gain command */
    if (received_message.rx_header.DLC == 8) {
        unpack_cmd(received_message, (float *)hfoc.mit_buf.commands);
        hfoc.new_cmd = 1;
    }
}
/* USER CODE END 1 */

