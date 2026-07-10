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
#include "PMSM_motor.h"
#include "foc_calibration.h"
#include "fsm.h"
#include "angle_sensor.h"
#include "analog_sensor.h"
#include "user_config.h"
#include "preference_writer.h"
#include "drv8353.h"
#include "can.h"
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
extern PMSM_motor       motor;
extern CalStruct        hcal;
extern PreferenceWriter prefs;

/* FOC loop timing instrumentation (set in HAL_ADCEx_InjectedConvCpltCallback) */
extern volatile uint32_t foc_loop_cycles_min;
extern volatile uint32_t foc_loop_cycles_max;
extern volatile uint32_t foc_loop_cycles_sum;
extern volatile uint32_t foc_loop_cycles_count;

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
  uint32_t timing_report_counter = 0;
  uint32_t temp_update_counter   = 0;
  for (;;)
  {
    osDelay(10);

    if (++temp_update_counter >= 100) {
      temp_update_counter = 0;
      update_temperature(&motor.motor_temp);
    }

    /* Print FOC-loop (run_fsm) timing stats once per 5 seconds, then reset */
    if (++timing_report_counter >= 500) {
      timing_report_counter = 0;
      float us_per_cycle = 1.0e6f / (float)SystemCoreClock;
      uint32_t cnt = foc_loop_cycles_count;
      if (cnt > 0) {
        printf("FOC loop:     min=%.2fus max=%.2fus avg=%.2fus (n=%lu)\r\n",
               foc_loop_cycles_min * us_per_cycle,
               foc_loop_cycles_max * us_per_cycle,
               (foc_loop_cycles_sum / (float)cnt) * us_per_cycle,
               (unsigned long)cnt);
        foc_loop_cycles_min   = 0xFFFFFFFFu;
        foc_loop_cycles_max   = 0;
        foc_loop_cycles_sum   = 0;
        foc_loop_cycles_count = 0;
      }
      drv_print_faults(motor.gateDriver);
    }

    if (pending_save) {
      pending_save = 0;
      osSemaphoreRelease(flash_write_sem);
    }

    if (motor.fsm.curr_state == SET_ZERO_MODE) {
      angle_sensor_set_m_zero(&motor.angle_sensor);
      CFG_M_ZERO_RAD = motor.angle_sensor.m_zero;
      osSemaphoreRelease(flash_write_sem);
      printf("Mechanical zero set: m_zero=%.4f rad\r\n", motor.angle_sensor.m_zero);
      motor.fsm.next_state = MENU_MODE;
    }

    if (hcal.cal_state == CAL_STATE_LUT_POSTPROC_PENDING) {
      foc_cal_lut_postprocess(&motor, &hcal);
      memcpy(&CFG_ENCODER_LUT, motor.angle_sensor.encd_error_comp,
             sizeof(motor.angle_sensor.encd_error_comp));
      CFG_E_ZERO_RAD   = motor.angle_sensor.e_zero;
      CFG_PHASE_ORDER = motor.angle_sensor.sensor_dir;
      CFG_PPAIRS = motor.angle_sensor.pole_pairs;
      CFG_CALIBRATION_DONE_FLAG = 1;
      osSemaphoreRelease(flash_write_sem);
      printf("LUT calibration complete, saved to flash\r\n");
    }

    if (motor.fsm.curr_state == ENCODER_MODE) {
      // printf("m_angle_raw:  %.4f rad\r\n", motor.angle_sensor.s_rotor_rad_raw);
      printf("raw_rad: %.4f rad\r\n", motor.angle_sensor.raw_rad);
    }

    // printf("id: %.3f\r\n",       motor.id);
    // printf("id_des: %.3f\r\n",   motor.id_ref);
    // printf("id_filt: %.3f\r\n",  motor.id_filtered);
    // printf("iq: %.3f\r\n",       motor.iq);
    // printf("i_q_des: %.3f\r\n",  motor.iq_ref);
    // printf("i_q_filt: %.3f\r\n", motor.iq_filtered);
    // printf("i_a: %.3f\r\n",      motor.current_sensor.ia_filtered);
    // printf("i_b: %.3f\r\n",      motor.current_sensor.ib_filtered);
    // printf("i_c: %.3f\r\n",      motor.current_sensor.ic_filtered);
    // printf("m_angle: %.4f\r\n",  motor.angle_sensor.s_rotor_rad);
    // printf("e_angle: %.4f\r\n",  motor.angle_sensor.e_rad);
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
    /* Reboot only after the write is durably committed, so init-consumed params
     * (encoder select) are picked up by a fresh motor_init. Safe because param
     * writes are only accepted in MENU_MODE, i.e. the motor is not driving. */
    if (pending_reboot) {
      pending_reboot = 0;
      NVIC_SystemReset();
    }
  }
  /* USER CODE END startFlashWritingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

