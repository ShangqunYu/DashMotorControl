/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Simple button press sends CAN message to computer
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

// CAN transmit structures
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];
uint32_t message_counter = 0;

// Button interrupt callback - runs when button pressed
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2)  // Button on PA2
    {
        // Prepare message: "BTN" + counter
        TxData[0] = 'B';
        TxData[1] = 'T';
        TxData[2] = 'N';
        TxData[3] = (message_counter >> 24) & 0xFF;
        TxData[4] = (message_counter >> 16) & 0xFF;
        TxData[5] = (message_counter >> 8) & 0xFF;
        TxData[6] = message_counter & 0xFF;
        TxData[7] = 0x00;
        
        // Send CAN message
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
        
        // Blink LED to show button was pressed
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);  // LED pin
        
        message_counter++;
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  
  /* USER CODE BEGIN 2 */

  // Start CAN peripheral
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  // Configure TX Header (message format)
  TxHeader.DLC = 8;                    // 8 bytes of data
  TxHeader.IDE = CAN_ID_STD;           // Standard 11-bit ID
  TxHeader.RTR = CAN_RTR_DATA;         // Data frame (not remote request)
  TxHeader.StdId = 0x100;              // CAN ID = 0x100
  TxHeader.TransmitGlobalTime = DISABLE;

  // Initialize message data
  for (int i = 0; i < 8; i++)
  {
      TxData[i] = 0;
  }

  // Send a startup message to show board is alive
  TxData[0] = 'R';
  TxData[1] = 'D';
  TxData[2] = 'Y';  // "RDY" = Ready
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
  
  // Blink LED to show we're ready
  for (int i = 0; i < 6; i++)
  {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
      HAL_Delay(100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Main loop does nothing - everything happens in button interrupt
    HAL_Delay(100);
    
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
      // Error - blink LED rapidly
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
      HAL_Delay(50);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif