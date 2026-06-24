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
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#include "fsm.h"
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
DRVStruct drv;
foc_t hfoc;
CalStruct hcal;
CANTxMessage can_tx;
CANRxMessage can_rx;
FSMStruct hfsm;

/* FOC loop timing instrumentation (DWT cycle counter) */
volatile uint32_t foc_loop_cycles_min = 0xFFFFFFFFu;
volatile uint32_t foc_loop_cycles_max = 0;
volatile uint32_t foc_loop_cycles_sum = 0;
volatile uint32_t foc_loop_cycles_count = 0;

/* angle_sensor_update timing instrumentation (DWT cycle counter) */
volatile uint32_t angle_sensor_cycles_min = 0xFFFFFFFFu;
volatile uint32_t angle_sensor_cycles_max = 0;
volatile uint32_t angle_sensor_cycles_sum = 0;
volatile uint32_t angle_sensor_cycles_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  I_MAX = 60.0f;
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 10000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  // if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 2.0f;}
  I_CAL=15.0f;
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  GR = 1.0f;
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  float KT_AFTER_REDUCER = 2.97f;
  KT = KT_AFTER_REDUCER/GR;
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(P_MAX)){P_MAX = 12.57f;}
  if(isnan(P_MIN)){P_MIN = -12.57f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Enable DWT cycle counter for FOC-loop timing instrumentation */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

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

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&CAN_H); //  Start CAN peripheral
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* DRV8353 setup */
  drv_init(drv, I_MAX);

  /* FOC sensor init — must come before MA732 start so the LUT is active
     from the very first SPI callback */
  foc_sensor_init(&hfoc,
      (CALIBRATION_DONE_FLAG == 1) ? E_ZERO_RAD        : 0.0f,
      (CALIBRATION_DONE_FLAG == 1 && PHASE_ORDER == 1) ? REVERSE_DIR : NORMAL_DIR);
  if (CALIBRATION_DONE_FLAG == 1) {
    hfoc.angle_sensor.m_zero     = isnan(M_ZERO_RAD) ? 0.0f : M_ZERO_RAD;
    hfoc.angle_sensor.pole_pairs = (uint8_t)PPAIRS;
    memcpy(hfoc.angle_sensor.encd_error_comp, &ENCODER_LUT,
           sizeof(hfoc.angle_sensor.encd_error_comp));
    hfoc.angle_sensor.lut_ready = 1;
    printf("Encoder cal loaded: e_zero=%.4f, ppairs=%d, dir=%s\r\n",
           hfoc.angle_sensor.e_zero, hfoc.angle_sensor.pole_pairs,
           (hfoc.angle_sensor.sensor_dir == REVERSE_DIR) ? "rev" : "norm");
  }

    // if using external encoder, set cs pin high for the internal one to disable it and avoid interference
  if(USE_EXTERNAL_ENCODER){
    HAL_GPIO_WritePin(ENC_CS_INT, GPIO_PIN_SET);
  }
  /* MA732 setup */
  MA732_config(&hfoc.angle_sensor.ma732, &ENC_SPI);
  for (int i=0; i<20; i++) {
    MA732_start(&hfoc.angle_sensor.ma732);
    HAL_Delay(10);
  }

  /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // shift ADC trigger to occur slightly before the PWM edge to allow for sampling during the deadtime
  htim1.Instance->CCR4 = htim1.Instance->ARR - ADC_TRIG_OFFSET;

  foc_timer_init(&hfoc, &htim1);
  foc_set_limit_current(&hfoc, I_MAX);
  init_trig_lut();

  pid_reset(&hfoc.id_ctrl);
  pid_set_ts(&hfoc.id_ctrl, FOC_TS);
  pid_set_kp(&hfoc.id_ctrl, 0.05f);
  pid_set_ki(&hfoc.id_ctrl, 200.0f);
  pid_set_max_out_dynamic(&hfoc.id_ctrl, 0.8f);
  pid_set_deadband(&hfoc.id_ctrl, 0.0001f);


  pid_reset(&hfoc.iq_ctrl);
  pid_set_ts(&hfoc.iq_ctrl, FOC_TS);
  pid_set_kp(&hfoc.iq_ctrl, 0.05f);
  pid_set_ki(&hfoc.iq_ctrl, 200.0f);
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
  hfsm.curr_state = MENU_MODE;
  hfsm.next_state = MENU_MODE;
  CurrentSensor_init(&hfoc.current_sensor, &(ADC1->JDR1), &(ADC2->JDR1), &(ADC3->JDR1), I_SCALE, 2048, 2048, 2048);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc3);
  HAL_Delay(50);

  drv_enable_gd(drv);
  htim1.Instance->CCR1 = 0u;
  htim1.Instance->CCR2 = 0u;
  htim1.Instance->CCR3 = 0u;
  CurrentSensor_calibrate(&hfoc.current_sensor, 1000U);
  printf("ADC offsets: A=%d, B=%d, C=%d\r\n",
         hfoc.current_sensor.adc_a_offset, hfoc.current_sensor.adc_b_offset, hfoc.current_sensor.adc_c_offset);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
// CAN RX is now handled entirely in CAN1_RX0_IRQHandler (stm32f4xx_it.c).
// Protocol: CAN_ID, 8 bytes.
//   0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC  → enter torque control (MOTOR_CMD)
//   0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD  → return to menu      (MENU_CMD)
//   0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE  → set mechanical zero  (ZERO_CMD)
//   otherwise                                → MIT pos/vel/kp/kd command
// Reply: 6 bytes, position/velocity/current packed as int16.

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == ENC_SPI.Instance)
  {
    // uint32_t cyc_start = DWT->CYCCNT;
    angle_sensor_update(&hfoc.angle_sensor);
    // uint32_t cycles = DWT->CYCCNT - cyc_start;
    // if (cycles < angle_sensor_cycles_min) angle_sensor_cycles_min = cycles;
    // if (cycles > angle_sensor_cycles_max) angle_sensor_cycles_max = cycles;
    // angle_sensor_cycles_sum += cycles;
    // angle_sensor_cycles_count++;
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
    // Don't trigger anything on the callbacks for ADC1 and ADC2.
    // We want to wait to do any work until we're sure we have info from
    // all three ADCs.
    CurrentSensor_sample_offset(&hfoc.current_sensor);

    uint32_t cyc_start = DWT->CYCCNT;
    run_fsm(&hfsm);
    uint32_t cycles = DWT->CYCCNT - cyc_start;

    if (cycles < foc_loop_cycles_min) foc_loop_cycles_min = cycles;
    if (cycles > foc_loop_cycles_max) foc_loop_cycles_max = cycles;
    foc_loop_cycles_sum += cycles;
    foc_loop_cycles_count++;
	}
  
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
