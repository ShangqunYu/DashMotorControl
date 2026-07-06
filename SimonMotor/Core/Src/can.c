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
#include "PMSM_motor.h"
#include "fsm.h"
#include <stdio.h>
#include <string.h>
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
CANRxMessage message_received;
CANTxMessage message_to_send;
uint32_t tx_mailbox;
extern PMSM_motor motor;
volatile uint8_t  pending_save = 0;

void init_can_rx_filter() {
    message_to_send.tx_header.DLC   = 8;
    message_to_send.tx_header.IDE   = CAN_ID_EXT;
    message_to_send.tx_header.RTR   = CAN_RTR_DATA;
    message_to_send.tx_header.ExtId = CFG_CAN_MASTER;

    CAN_FilterTypeDef filter;
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.FilterBank           = 10;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh         = CFG_CAN_ID >> 13 & 0xFFFF;
    filter.FilterIdLow          = CFG_CAN_ID << 3 & 0xFFF8;
    filter.FilterMaskIdHigh     = FILTER_MASK >> 13 & 0xFFFF;
    filter.FilterMaskIdLow      = FILTER_MASK << 3 & 0xFFF8;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}

void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb, float temp){
    int p_int = float_to_uint(p, CFG_P_MIN, CFG_P_MAX, 16);
    int v_int = float_to_uint(v, CFG_V_MIN, CFG_V_MAX, 12);
    int t_int = float_to_uint(t, -CFG_I_MAX*CFG_KT*CFG_GR, CFG_I_MAX*CFG_KT*CFG_GR, 12);
    int vb_int = float_to_uint(vb, V_BUS_MIN, V_BUS_MAX, 8);
    int temp_int = float_to_uint(temp, CFG_TEMP_MIN, CFG_TEMP_MAX, 8);
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

    commands[0] = uint_to_float(p_int, CFG_P_MIN, CFG_P_MAX, 16);
    commands[1] = uint_to_float(v_int, CFG_V_MIN, CFG_V_MAX, 12);
    commands[2] = uint_to_float(kp_int, 0, CFG_KP_MAX, 12);
    commands[3] = uint_to_float(kd_int, 0, CFG_KD_MAX, 12);
    commands[4] = uint_to_float(t_int, -CFG_I_MAX*CFG_KT*CFG_GR, CFG_I_MAX*CFG_KT*CFG_GR, 12);
}

static void pack_param_reply(CANTxMessage *msg, uint8_t id, uint8_t param_index) {
    float value = 0.0f;
    user_config_get_param((param_id_t)param_index, &value);
    uint32_t bits;
    memcpy(&bits, &value, 4);
    msg->data[0] = id;
    msg->data[1] = param_index;
    msg->data[2] = (bits >> 24) & 0xFF;
    msg->data[3] = (bits >> 16) & 0xFF;
    msg->data[4] = (bits >>  8) & 0xFF;
    msg->data[5] =  bits        & 0xFF;
    msg->data[6] = 0;
    msg->data[7] = 0;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &message_received.rx_header, message_received.data);

    uint8_t *d   = message_received.data;
    int      dlc = message_received.rx_header.DLC;

    /* [FF×7][cmd] — mode switch, always allowed */
    if (d[0]==0xFF && d[1]==0xFF && d[2]==0xFF && d[3]==0xFF && d[4]==0xFF && d[5]==0xFF && d[6]==0xFF) {
        if (d[7] <= L_MEAS_MODE) motor.cmd.pending_fsm_cmd = d[7];
        return;
    }

    if (motor.fsm.curr_state == MENU_MODE) {
        /* [FF×6][FE][idx] — param read */
        if (d[0]==0xFF && d[1]==0xFF && d[2]==0xFF && d[3]==0xFF && d[4]==0xFF && d[5]==0xFF && d[6]==0xFE) {
            pack_param_reply(&message_to_send, CFG_CAN_ID, d[7]);
            HAL_CAN_AddTxMessage(&CAN_H, &message_to_send.tx_header, message_to_send.data, &tx_mailbox);
            return;
        }
        /* [FF][FF][FD][idx][b3 b2 b1 b0] — param write; idx=0xFF → factory reset */
        if (d[0]==0xFF && d[1]==0xFF && d[2]==0xFD) {
            uint8_t idx = d[3];
            if (idx == 0xFF) {
                user_config_set_defaults();
            } else {
                uint32_t bits = ((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16)
                              | ((uint32_t)d[6] <<  8) |  (uint32_t)d[7];
                float value;
                memcpy(&value, &bits, 4);
                user_config_set_param((param_id_t)idx, value);
            }
            pending_save = 1;
            pack_param_reply(&message_to_send, CFG_CAN_ID, idx);
            HAL_CAN_AddTxMessage(&CAN_H, &message_to_send.tx_header, message_to_send.data, &tx_mailbox);
            return;
        }
    }

    /* MIT position/velocity/gain command */
    pack_reply(&message_to_send, CFG_CAN_ID, motor.angle_sensor.mech_angle_rad,
               motor.angle_sensor.mech_angle_vel, motor.iq*CFG_KT*CFG_GR, motor.v_bus, motor.motor_temp);
    HAL_CAN_AddTxMessage(&CAN_H, &message_to_send.tx_header, message_to_send.data, &tx_mailbox);
    if (dlc == 8) {
        unpack_cmd(message_received, (float *)motor.cmd.cmd_buf.commands);
        motor.cmd.new_cmd = 1;
    }
}
/* USER CODE END 1 */

