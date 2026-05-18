/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "foc_calibration.h"
#include "angle_sensor.h"
#include "user_config.h"
#include "preference_writer.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern foc_t       hfoc;
extern CalStruct   hcal;
extern PreferenceWriter prefs;

static osSemaphoreId_t flash_write_sem;
/* USER CODE END Variables */
/* Definitions for motorMgrTask */
osThreadId_t motorMgrTaskHandle;
const osThreadAttr_t motorMgrTask_attributes = {
  .name = "motorMgrTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for flashWritingTas */
osThreadId_t flashWritingTasHandle;
const osThreadAttr_t flashWritingTas_attributes = {
  .name = "flashWritingTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void startMotorMgrTask(void *argument);
void startFlashWritingTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  flash_write_sem = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorMgrTask */
  motorMgrTaskHandle = osThreadNew(startMotorMgrTask, NULL, &motorMgrTask_attributes);

  /* creation of flashWritingTas */
  flashWritingTasHandle = osThreadNew(startFlashWritingTask, NULL, &flashWritingTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_startMotorMgrTask */
/**
  * @brief  Function implementing the motorMgrTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startMotorMgrTask */
void startMotorMgrTask(void *argument)
{
  /* USER CODE BEGIN startMotorMgrTask */
  for (;;)
  {
    osDelay(10);

    if (hfoc.control_mode == SET_ZERO_MODE) {
      angle_sensor_set_m_zero(&hfoc.angle_sensor);
      M_ZERO_RAD = hfoc.angle_sensor.m_zero;
      osSemaphoreRelease(flash_write_sem);
      printf("Mechanical zero set: m_zero=%.4f rad\r\n", hfoc.angle_sensor.m_zero);
      hfoc.control_mode = ENCODER_MODE;
    }

    if (hcal.cal_state == CAL_STATE_LUT_POSTPROC_PENDING) {
      foc_cal_lut_postprocess(&hfoc, &hcal);
      memcpy(&ENCODER_LUT, hfoc.angle_sensor.encd_error_comp,
             sizeof(hfoc.angle_sensor.encd_error_comp));
      E_ZERO_RAD   = hfoc.angle_sensor.e_zero;
      CALIBRATION_DONE_FLAG = 1;
      osSemaphoreRelease(flash_write_sem);
      printf("LUT calibration complete, saved to flash\r\n");
    }

    if (hfoc.control_mode == ENCODER_MODE) {
      printf("m_angle_raw:  %.4f rad\r\n", hfoc.angle_sensor.m_angle_rad_raw);
      printf("m_angle_comp: %.4f rad\r\n", hfoc.angle_sensor.m_angle_rad);
    }
  }
  /* USER CODE END startMotorMgrTask */
}

/* USER CODE BEGIN Header_startFlashWritingTask */
/**
* @brief Function implementing the flashWritingTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startFlashWritingTask */
void startFlashWritingTask(void *argument)
{
  /* USER CODE BEGIN startFlashWritingTask */
  for (;;)
  {
    // Block indefinitely until motor_mgr_task signals that flash needs updating
    osSemaphoreAcquire(flash_write_sem, osWaitForever);
    if (!preference_writer_ready(prefs)) { preference_writer_open(&prefs); }
    preference_writer_flush(&prefs);
    preference_writer_close(&prefs);
    preference_writer_load(prefs);
  }
  /* USER CODE END startFlashWritingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

