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
#include "MA732.h"
#include "foc.h"
#include "pid_utils.h"
#define BLDC_PWM_FREQ 10000
#define FOC_TS (1.0f / (float)BLDC_PWM_FREQ)
#define SPEED_CONTROL_CYCLE	10
#define SPEED_TS (FOC_TS * SPEED_CONTROL_CYCLE) 
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

/* USER CODE BEGIN PV */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;
static MotorControlPid_t motor_pid;
DRVStruct drv;
foc_t hfoc;


static volatile uint16_t encoder_last_word = 0U;

bool pose_ready = false;
float rpm = 0.0f;
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

  switch(hfoc.control_mode) {
    case TORQUE_CONTROL_MODE:
      hfoc.id_ref = 0.0f;
      hfoc.iq_ref = 1.0f; // Set a constant torque reference for testing
      foc_current_control_update(&hfoc, FOC_TS);
      break;
    case SPEED_CONTROL_MODE:
      hfoc.rpm_ref = 60.0f;
      foc_speed_control_update(&hfoc, hfoc.rpm_ref);
      foc_current_control_update(&hfoc, FOC_TS);
      break;
    case MIT_MODE: {
      static uint32_t mit_tick = 0;
      const float freq = 0.5f;  // Hz - one full oscillation every 2 s
      const uint32_t period_ticks = (uint32_t)(1.0f / (freq * FOC_TS));
      float t = (float)mit_tick * FOC_TS;
      if (++mit_tick >= period_ticks) mit_tick = 0;

      hfoc.mit_cmd.kp = 20.0f;
      hfoc.mit_cmd.kd = 0.02f;
      // Sine between 0° and 180°: center 90°, amplitude 90°
      hfoc.mit_cmd.des_pos = PI / 2.0f + PI / 2.0f * fast_sin(TWO_PI * freq * t);
      // Feedforward velocity (rad/s → RPM): d(des_pos)/dt * 60/(2π)
      hfoc.mit_cmd.des_vel = 30.0f * PI * freq * fast_cos(TWO_PI * freq * t);
      hfoc.mit_cmd.f_tau = 0.0f;
      foc_mit_control_update(&hfoc);
      foc_current_control_update(&hfoc, FOC_TS);
      break;
    }
    case POWER_UP_MODE:
      // open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
      break;
    case CALIBRATION_MODE:
      foc_cal_encoder_misalignment_update(&hfoc, FOC_TS);
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
  // if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  // {
	//   Error_Handler();
  // }


    /* DRV8323 setup */
  HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET ); 	// CS high
  HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET );
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  HAL_Delay(1);
  drv_write_HSR(drv, LOCK_OFF, IDRIVEP_HS_300MA, IDRIVEN_HS_200MA);
  HAL_Delay(1);
  drv_write_LSR(drv, 1, TDRIVE_1000NS, IDRIVEP_LS_850MA, IDRIVEN_LS_600MA);
  HAL_Delay(1);
  int CSA_GAIN;
  if(I_MAX <= 40.0f){CSA_GAIN = CSA_GAIN_40;}	// Up to 40A use 40X amplifier gain
  else{CSA_GAIN = CSA_GAIN_20;}					// From 40-60A use 20X amplifier gain.  (Make this generic in the future)
//  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_0_25);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN, 0x1, 0x0, 0x0, 0x0, SEN_LVL_0_5);
  HAL_Delay(1);
  // zero_current(&controller);
  HAL_Delay(1);
  drv_write_OCPCR(drv, TRETRY_8MS, DEADTIME_50NS, OCP_LATCH, OCP_DEG_4US, VDS_LVL_0_7);
  HAL_Delay(1);
  drv_disable_gd(drv);
  HAL_Delay(1);

  MA732_config(&hfoc.ma732, &ENC_SPI);
  for (int i=0; i<20; i++) {
    MA732_start(&hfoc.ma732);
    HAL_Delay(10);
  }
    // MA732_start(&hfoc.ma732);

    /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // the total adc reading takes about 5us, so set the offset to 2.5us to allow for some margin. 
  // uint32_t offset = (uint32_t)((htim1.Instance->ARR+ 1) * 2  / 180 * 2.5);
  uint32_t offset = (uint32_t)(2.5f * 180);
  htim1.Instance->CCR4 = htim1.Instance->ARR - offset;

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
  hfoc.control_mode = CALIBRATION_MODE;
  foc_cal_encoder_misalignment_start(&hfoc);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_Delay(10);
    drv_print_faults(drv);
    HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
    printf("i_a: %.3f\r\n", hfoc.current_sensor.ia_filtered);
    printf("i_b: %.3f\r\n", hfoc.current_sensor.ib_filtered);
    printf("i_c: %.3f\r\n", hfoc.current_sensor.ic_filtered);
    printf("id: %.3f\r\n", hfoc.id);
    printf("id_des: %.3f\r\n", hfoc.id_ref);
    printf("id_filt: %.3f\r\n", hfoc.id);
    printf("iq: %.3f\r\n", hfoc.iq);
    printf("i_q_des: %.3f\r\n", hfoc.iq_ref);
    printf("i_q_filt: %.3f\r\n", hfoc.iq);
    printf("m_angle: %.3f\r\n", hfoc.m_angle_rad);
    printf("des_pos: %.3f\r\n", hfoc.mit_cmd.des_pos);
    printf("e_angle: %.3f\r\n", hfoc.e_angle_rad);
    printf("rpm: %.3f\r\n", hfoc.actual_rpm);
    printf("rpm_ref: %.3f\r\n", hfoc.rpm_ref);

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
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == ENC_SPI.Instance)
  {
    pose_ready = true;
    foc_sensored_calc_electric_angle(&hfoc);
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
