/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRV_MOSI_Pin GPIO_PIN_13
#define DRV_MOSI_GPIO_Port GPIOC
#define MOTOR_ENABLE_Pin GPIO_PIN_14
#define MOTOR_ENABLE_GPIO_Port GPIOC
#define MOTOR_HIZ_Pin GPIO_PIN_15
#define MOTOR_HIZ_GPIO_Port GPIOC
#define DRV_CS_Pin GPIO_PIN_0
#define DRV_CS_GPIO_Port GPIOB
#define ENC_CS_Pin GPIO_PIN_2
#define ENC_CS_GPIO_Port GPIOB
#define MOTOR_FAULT_Pin GPIO_PIN_13
#define MOTOR_FAULT_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOC
#define DRV_SCLK_Pin GPIO_PIN_10
#define DRV_SCLK_GPIO_Port GPIOC
#define DRV_MISO_Pin GPIO_PIN_11
#define DRV_MISO_GPIO_Port GPIOC
#define ENC_CS2_Pin GPIO_PIN_7
#define ENC_CS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
