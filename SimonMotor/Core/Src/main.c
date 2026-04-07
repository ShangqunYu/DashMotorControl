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
#define BLDC_PWM_FREQ 10000
#define FOC_TS (1.0f / (float)BLDC_PWM_FREQ)
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
static float motor_pid_output = 0.0f;
DRVStruct drv;
static union
{
    uint8_t buff[2];
    uint16_t word;
} encoder_spi_tx, encoder_spi_rx;
static volatile uint16_t encoder_last_word = 0U;

bool pose_ready = false;
float angle_deg = 0.0f;
float rpm = 0.0f;
float voltage = 0.0f;
float I_u = 0.0f;
float I_v = 0.0f;
float I_w = 0.0f;
float i_scale = 0.0201416f;
int adc_a_offset = 2048;
int adc_b_offset = 2048;
static volatile uint8_t adc_offset_calibrating = 0U;
static volatile uint32_t adc_a_offset_sum = 0U;
static volatile uint32_t adc_b_offset_sum = 0U;
static volatile uint32_t adc_offset_sample_count = 0U;

MA732_t encd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void startEncoderRead(void)
{
  if (HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY)
  {
    return;
  }

  encoder_spi_tx.word = ENC_READ_WORD;
  HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(&ENC_SPI,
                              (uint8_t *)&encoder_spi_tx.buff,
                              (uint8_t *)&encoder_spi_rx.buff,
                              1);
}


float get_power_voltage(void) {
	static float pv_filtered = 0.0f;
  const float filter_alpha = 0.2f; 

	// convert to volt
	float pv = (float)ADC3->JDR1 * V_SCALE;

  // Low-pass filter for noise reduction
	pv_filtered = (1.0f - filter_alpha) * pv_filtered + filter_alpha * pv;

	return pv_filtered;
}

void get_current(void){
  float i_u = ((float)ADC1->JDR1 - adc_a_offset) * i_scale;
  float i_v = ((float)ADC2->JDR1 - adc_b_offset) * i_scale;
  float i_w = -i_u - i_v; // Assuming sum of currents is zero

  I_u = i_u;
  I_v = i_v;
  I_w = i_w;

}

void calibrate_adc_offsets(uint32_t duration_ms)
{
  uint32_t start_tick = HAL_GetTick();

  adc_offset_calibrating = 1U;
  adc_a_offset_sum = 0U;
  adc_b_offset_sum = 0U;
  adc_offset_sample_count = 0U;

  while ((HAL_GetTick() - start_tick) < duration_ms)
  {
    HAL_Delay(1);
  }

  adc_offset_calibrating = 0U;

  if (adc_offset_sample_count > 0U)
  {
    adc_a_offset = (int)(adc_a_offset_sum / adc_offset_sample_count);
    adc_b_offset = (int)(adc_b_offset_sum / adc_offset_sample_count);
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

  MA732_config(&encd, &ENC_SPI);
  MA732_start(&encd);
  HAL_Delay(10);
  MA732_start(&encd);

    /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // the total adc reading takes about 5us, so set the offset to 2.5us to allow for some margin. 
  // uint32_t offset = (uint32_t)((htim1.Instance->ARR+ 1) * 2  / 180 * 2.5);
  uint32_t offset = (uint32_t)(2.5f * 180);
  htim1.Instance->CCR4 = htim1.Instance->ARR - offset;
  // set PWM duty cycle to 90% to test
  // htim1.Instance->CCR1 = (uint32_t)(htim1.Instance->ARR * 0.95f);
  // htim1.Instance->CCR2 = (uint32_t)(htim1.Instance->ARR * 0.95f);
  // htim1.Instance->CCR3 = (uint32_t)(htim1.Instance->ARR * 0.95f);

  	// current sensor
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	// voltage sensor
	HAL_ADCEx_InjectedStart_IT(&hadc3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  drv_enable_gd(drv);
  htim1.Instance->CCR1 = (uint32_t)(htim1.Instance->ARR * 0.0f);
  htim1.Instance->CCR2 = (uint32_t)(htim1.Instance->ARR * 0.0f);
  htim1.Instance->CCR3 = (uint32_t)(htim1.Instance->ARR * 0.0f);
  calibrate_adc_offsets(1000U);
  printf("ADC offsets: A=%d, B=%d\r\n", adc_a_offset, adc_b_offset);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t ramp_tick = HAL_GetTick() % 20000U;
    uint32_t max_compare = htim1.Init.Period / 64U;
    uint32_t compare_value;

    // motor_pid_output = MotorControl_UpdatePid(&motor_pid, 1000.0f, 950.0f, 0.001f);
    HAL_Delay(10);
    drv_print_faults(drv);
    // htim1.Instance->CCR1 = (uint32_t)(htim1.Instance->ARR * 0.4f);
    // htim1.Instance->CCR2 = (uint32_t)(htim1.Instance->ARR * 0.1f);
    // htim1.Instance->CCR3 = (uint32_t)(htim1.Instance->ARR * 0.1f);
    if (ramp_tick < 10000U)
    {
      compare_value = (max_compare * ramp_tick) / 10000U;
    }
    else
    {
      compare_value = (max_compare * (20000U - ramp_tick)) / 10000U;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare_value);
    // printf("spi_rx_buff: %u\r\n", encoder_last_word);
    if (pose_ready)
    {
      HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
      // encoder_last_word = encoder_spi_rx.word & 0xFFF0;
      printf("Angle: %.2f\r\n", angle_deg);
      printf("RPM: %.2f\r\n", rpm);
      printf("Voltage: %.2f\r\n", voltage);
      printf("Current U: %.2f A\r\n", I_u);
      printf("Current V: %.2f A\r\n", I_v);
      printf("Current W: %.2f A\r\n", I_w);
      pose_ready = false;
      // startEncoderRead();

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
    angle_deg = MA732_get_degree(&encd);
    pose_ready = true;
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
    if (adc_offset_calibrating != 0U)
    {
      adc_a_offset_sum += ADC1->JDR1;
      adc_b_offset_sum += ADC2->JDR1;
      adc_offset_sample_count++;
    }
    rpm = MA732_get_rpm(&encd, FOC_TS);
    MA732_start(&encd);
    voltage= get_power_voltage();
    get_current();
	}
  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
  // printf("ADC3 injected conversion complete\r\n");
  // flip PA_4
  
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
