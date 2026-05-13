/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
#include "stm32f4xx_flash.h"
#include "flash_writer.h"
#include "preference_writer.h"
#include "drv8353.h"
#include <stdio.h>
#include <string.h>
#include "foc.h"
#include "angle_sensor.h"
#include "foc_calibration.h"
#include "pid_utils.h"
#define BLDC_PWM_FREQ 40000
#define FOC_TS (1.0f / (float)BLDC_PWM_FREQ)
#define SPEED_CONTROL_CYCLE	10
#define SPEED_TS (FOC_TS * SPEED_CONTROL_CYCLE) 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// CAN command buffer – written by CAN ISR, consumed by foc_loop
typedef struct {
    motor_mode_t mode;
    float        des_pos;
    float        des_vel;
    float        kp;
    float        kd;
    uint8_t      mode_pending;   // 1 = new mode waiting
    uint8_t      mit_pending;    // 1 = new MIT params waiting
} can_cmd_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;
static MotorControlPid_t motor_pid;
DRVStruct drv;
foc_t hfoc;
CalStruct hcal;


static volatile uint16_t encoder_last_word = 0U;

bool pose_ready = false;
float rpm = 0.0f;

volatile can_cmd_t g_can_cmd = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


float get_power_voltage(void) {
	static float pv_filtered = 0.0f;
  const float filter_alpha = 0.2f; 

	// convert to volt
	float pv = (float)ADC3->JDR1 * V_SCALE;

  // Low-pass filter for noise reduction
	pv_filtered = (1.0f - filter_alpha) * pv_filtered + filter_alpha * pv;

	return pv_filtered;
}


static void foc_loop(void) {

  hfoc.v_bus = get_power_voltage();
  CurrentSensor_sample_offset(&hfoc.current_sensor);

  // Update position and velocity (called every FOC cycle)
  foc_update_position_velocity(&hfoc, FOC_TS);
  // // pa4 set high
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  // // pa4 set low
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);


  // Apply any pending CAN command before running the control loop
  if (g_can_cmd.mode_pending) {
    motor_mode_t new_mode = g_can_cmd.mode;
    g_can_cmd.mode_pending = 0;
    hfoc.control_mode = new_mode;
    if (new_mode == CALIBRATION_MODE) {
      foc_cal_encoder_misalignment_start(&hfoc, &hcal);
    }
  }
  if (g_can_cmd.mit_pending) {
    hfoc.mit_cmd.des_pos = g_can_cmd.des_pos;
    hfoc.mit_cmd.des_vel = g_can_cmd.des_vel;
    hfoc.mit_cmd.kp      = g_can_cmd.kp;
    hfoc.mit_cmd.kd      = g_can_cmd.kd;
    g_can_cmd.mit_pending = 0;
  }

  switch(hfoc.control_mode) {
    case TORQUE_CONTROL_MODE:
      hfoc.id_ref = 0.0f;
      hfoc.iq_ref = 1.0f; // Set a constant torque reference for testing
      foc_current_control_update(&hfoc, FOC_TS);
      break;
    case SPEED_CONTROL_MODE:
      hfoc.vel_ref = TWO_PI;  // 1 rev/s = 2π rad/s (~60 RPM)
      foc_speed_control_update(&hfoc, hfoc.vel_ref);
      foc_current_control_update(&hfoc, FOC_TS);
      break;
    case MIT_MODE: {
      static uint32_t mit_tick = 0;
      static uint32_t period_start_ms = 0;
      const float freq = 1.0f;  // Hz - one full oscillation every 1 s
      const uint32_t period_ticks = (uint32_t)(1.0f / (freq * FOC_TS));
      float t = (float)mit_tick * FOC_TS;
      if (++mit_tick >= period_ticks) {
        mit_tick = 0;
        uint32_t now_ms = HAL_GetTick();
        // printf("period elapsed: %lu ms\n", now_ms - period_start_ms);
        period_start_ms = now_ms;
      }

      hfoc.mit_cmd.kp = 20.0f;
      hfoc.mit_cmd.kd = 0.2f;
      // Sine between 0° and 180°: center 90°, amplitude 90°
      hfoc.mit_cmd.des_pos = PI / 2.0f + PI / 2.0f * fast_sin(TWO_PI * freq * t);
      // Feedforward velocity (rad/s):
      hfoc.mit_cmd.des_vel = 0.0f;
      hfoc.mit_cmd.f_tau = 0.0f;
      foc_mit_control_update(&hfoc);
      foc_current_control_update(&hfoc, FOC_TS);
      break;
    }
    case ENCODER_MODE:
      // Motor coasts; angle is updated by the SPI interrupt.
      // Raw vs compensated comparison is printed in the main loop.
      open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
      break;
    case POWER_UP_MODE:
      // open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
      break;
    case CALIBRATION_MODE:
      foc_cal_encoder_misalignment_update(&hfoc, &hcal, FOC_TS);
      break;
    default:
      break;
  }
  
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  preference_writer_init(&prefs, 6);
  preference_writer_load(prefs);
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 10000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 2.0f;}
  I_CAL=5.0f;
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(P_MAX)){P_MAX = 12.5f;}
  if(isnan(P_MIN)){P_MIN = -12.5f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MotorControl_InitPid(&motor_pid, 1.0f, 0.1f, 0.01f, -100.0f, 100.0f);
  HAL_CAN_Start(&CAN_H); //  Start CAN peripheral
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }


    /* DRV8323 setup */
  drv_init(drv, I_MAX);

  MA732_config(&hfoc.angle_sensor.ma732, &ENC_SPI);
  for (int i=0; i<20; i++) {
    MA732_start(&hfoc.angle_sensor.ma732);
    HAL_Delay(10);
  }
    // MA732_start(&hfoc.angle_sensor.ma732);

    /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // the total adc reading takes about 5us, so set the offset to 2.5us to allow for some margin. 
  // uint32_t offset = (uint32_t)((htim1.Instance->ARR+ 1) * 2  / 180 * 2.5);
  uint32_t offset = (uint32_t)(2.5f * 180);
  htim1.Instance->CCR4 = htim1.Instance->ARR - 240;

  foc_motor_init(&hfoc, POLE_PAIR, 360.0f);
  foc_sensor_init(&hfoc, 0.0f, NORMAL_DIR);
  foc_timer_init(&hfoc, &htim1);
  foc_set_limit_current(&hfoc, 20.0f);
  init_trig_lut();

  pid_reset(&hfoc.id_ctrl);
  pid_set_ts(&hfoc.id_ctrl, FOC_TS);
  pid_set_kp(&hfoc.id_ctrl, 0.2f);
  pid_set_ki(&hfoc.id_ctrl, 5.0f);
  pid_set_max_out_dynamic(&hfoc.id_ctrl, 0.8f);
  pid_set_deadband(&hfoc.id_ctrl, 0.0001f);


  pid_reset(&hfoc.iq_ctrl);
  pid_set_ts(&hfoc.iq_ctrl, FOC_TS);
  pid_set_kp(&hfoc.iq_ctrl, 0.2f);
  pid_set_ki(&hfoc.iq_ctrl, 5.0f);
  pid_set_max_out_dynamic(&hfoc.iq_ctrl, 0.8f);
  pid_set_deadband(&hfoc.iq_ctrl, 0.0001f);

  // Speed PID parameter
  pid_reset(&hfoc.speed_ctrl);
  pid_set_ts(&hfoc.speed_ctrl, SPEED_TS);
  pid_set_kp(&hfoc.speed_ctrl, 0.01f);
  pid_set_ki(&hfoc.speed_ctrl, 0.1f);
  pid_set_kd(&hfoc.speed_ctrl, 0.0001f);
  pid_set_d_filter_fc(&hfoc.speed_ctrl, 100.0f);
  pid_set_max_d(&hfoc.speed_ctrl, 10.0f);
  pid_set_max_out(&hfoc.speed_ctrl, 10.0f);
  pid_set_deadband(&hfoc.speed_ctrl, 0.01f);

  	// current sensor
  hfoc.control_mode = POWER_UP_MODE;
  CurrentSensor_init(&hfoc.current_sensor, &(ADC1->JDR1), &(ADC2->JDR1), I_SCALE, 2048, 2048);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	// voltage sensor
	HAL_ADCEx_InjectedStart_IT(&hadc3);
  HAL_Delay(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  drv_enable_gd(drv);
  htim1.Instance->CCR1 = (uint32_t)(htim1.Instance->ARR * 0.0f);
  htim1.Instance->CCR2 = (uint32_t)(htim1.Instance->ARR * 0.0f);
  htim1.Instance->CCR3 = (uint32_t)(htim1.Instance->ARR * 0.0f);
  CurrentSensor_calibrate(&hfoc.current_sensor, 1000U);
  printf("ADC offsets: A=%d, B=%d\r\n", hfoc.current_sensor.adc_a_offset, hfoc.current_sensor.adc_b_offset);

  // Load encoder calibration from flash if a previous calibration was saved
  if (CALIBRATION_DONE_FLAG == 1) {
    hfoc.angle_sensor.m_angle_offset = M_ANGLE_OFFSET;
    memcpy(hfoc.angle_sensor.encd_error_comp, &ENCODER_LUT,
           sizeof(hfoc.angle_sensor.encd_error_comp));
    hfoc.angle_sensor.lut_ready = 1;
    printf("Encoder cal loaded: offset=%.4f rad\r\n", hfoc.angle_sensor.m_angle_offset);
  }

  hfoc.control_mode = POWER_UP_MODE;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_Delay(10);

    // Post-process LUT after sweeps finish (1024×128 ops – must run outside ISR)
    if (hcal.cal_state == CAL_STATE_LUT_POSTPROC_PENDING) {
      foc_cal_lut_postprocess(&hfoc, &hcal);

      // Persist LUT and angle offset to flash
      memcpy(&ENCODER_LUT, hfoc.angle_sensor.encd_error_comp,
             sizeof(hfoc.angle_sensor.encd_error_comp));
      M_ANGLE_OFFSET = hfoc.angle_sensor.m_angle_offset;
      CALIBRATION_DONE_FLAG = 1;
      if (!preference_writer_ready(prefs)) { preference_writer_open(&prefs); }
      preference_writer_flush(&prefs);
      preference_writer_close(&prefs);
      preference_writer_load(prefs);
      printf("LUT calibration complete, saved to flash\r\n");
    }

    // drv_print_faults(drv);
    HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);

    if (hfoc.control_mode == ENCODER_MODE) {
      printf("m_angle_raw: %.4f\r\n", hfoc.angle_sensor.m_angle_rad_raw);
      printf("m_angle_comp: %.4f\r\n", hfoc.angle_sensor.m_angle_rad);
    }
    // printf("i_a: %.3f\r\n", hfoc.current_sensor.ia_filtered);
    // printf("i_b: %.3f\r\n", hfoc.current_sensor.ib_filtered);
    // printf("i_c: %.3f\r\n", hfoc.current_sensor.ic_filtered);
    // printf("id: %.3f\r\n", hfoc.id);
    // printf("id_des: %.3f\r\n", hfoc.id_ref);
    // printf("id_filt: %.3f\r\n", hfoc.id);
    // printf("iq: %.3f\r\n", hfoc.iq);
    // printf("i_q_des: %.3f\r\n", hfoc.iq_ref);
    // printf("i_q_filt: %.3f\r\n", hfoc.iq);
    // printf("m_angle: %.3f\r\n", hfoc.angle_sensor.m_angle_rad);
    // printf("des_pos: %.3f\r\n", hfoc.mit_cmd.des_pos);
    // printf("e_angle: %.3f\r\n", hfoc.angle_sensor.e_angle_rad);
    // printf("vel: %.3f\r\n", hfoc.angle_sensor.actual_vel);
    // printf("vel_ref: %.3f\r\n", hfoc.vel_ref);
    if (pose_ready)
    {

      pose_ready = false;

    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// CAN protocol
//  ID = CAN_ID   : mode switch  – data[0] = motor_mode_t value
//  ID = CAN_ID+1 : MIT command  – 8 bytes
//    [0..1] int16  des_pos  scaled over [P_MIN .. P_MAX]   (rad)
//    [2..3] int16  des_vel  scaled over [V_MIN .. V_MAX]   (rad/s)
//    [4..5] uint16 kp       scaled over [0 .. KP_MAX]
//    [6..7] uint16 des_kd   scaled over [0 .. KD_MAX]
//
// Scaling helpers (matches MIT mini-cheetah convention):
//   float = lo + (int16 + 32768) / 65535 * (hi - lo)
static inline float scale_int16(int16_t raw, float lo, float hi)
{
    return lo + ((float)(raw + 32768) / 65535.0f) * (hi - lo);
}
static inline float scale_uint16(uint16_t raw, float hi)
{
    return ((float)raw / 65535.0f) * hi;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK)
        return;

    uint32_t id = hdr.ExtId;

    if (id == (uint32_t)CAN_ID) {
        // Mode switch: single byte
        g_can_cmd.mode         = (motor_mode_t)data[0];
        g_can_cmd.mode_pending = 1;
    }
    else if (id == (uint32_t)(CAN_ID + 1) && hdr.DLC == 8) {
        // MIT command: 8-byte packed
        int16_t  raw_pos = (int16_t)((data[0] << 8) | data[1]);
        int16_t  raw_vel = (int16_t)((data[2] << 8) | data[3]);
        uint16_t raw_kp  = (uint16_t)((data[4] << 8) | data[5]);
        uint16_t raw_kd  = (uint16_t)((data[6] << 8) | data[7]);

        g_can_cmd.des_pos    = scale_int16(raw_pos, P_MIN, P_MAX);
        g_can_cmd.des_vel    = scale_int16(raw_vel, V_MIN, V_MAX);
        g_can_cmd.kp         = scale_uint16(raw_kp, KP_MAX);
        g_can_cmd.kd         = scale_uint16(raw_kd, KD_MAX);
        g_can_cmd.mit_pending = 1;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == ENC_SPI.Instance)
  {
    pose_ready = true;
    angle_sensor_update(&hfoc.angle_sensor);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == ENC_SPI.Instance)
  {
    HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
  }
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
  }
	if (hadc->Instance == ADC2) {
  }
	if (hadc->Instance == ADC3) {
    foc_loop();
	}
  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
