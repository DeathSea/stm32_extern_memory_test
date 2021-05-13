/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "command_set.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void WREN()
{
  QSPI_CommandTypeDef command;
  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = WRITE_ENABLE;
  command.Address = 0;
  command.AddressSize = 0;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.AlternateBytesSize = 0;
  command.AlternateBytes = 0;
  command.DummyCycles = 0;
  command.DataMode = QSPI_DATA_NONE;
  command.NbData = 0;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

void quad_enable()
{
  QSPI_CommandTypeDef command;
  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = WRITE_STATUS_REG;
  command.Address = 0;
  command.AddressSize = 0;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.AlternateBytesSize = 0;
  command.AlternateBytes = 0;
  command.DummyCycles = 0;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 1;

  if (HAL_QSPI_Command(&hqspi, &command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

// status reg SRWD|QE|BP3|BP2|BP1|BP0|WEL|WIP
void set_polling_config_WEL(QSPI_AutoPollingTypeDef* pollingConfig)
{
  pollingConfig->Match = 0x02; 
  pollingConfig->Mask = 0x02;
  pollingConfig->Interval = 1;
  pollingConfig->StatusBytesSize = 1;
  pollingConfig->MatchMode = QSPI_MATCH_MODE_AND;
  pollingConfig->AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
}

// status reg SRWD|QE|BP3|BP2|BP1|BP0|WEL|WIP
void set_polling_config_WIP(QSPI_AutoPollingTypeDef* pollingConfig)
{
  pollingConfig->Match = 0x00;
  pollingConfig->Mask = 0x01;
  pollingConfig->Interval = 1;
  pollingConfig->StatusBytesSize = 1;
  pollingConfig->MatchMode = QSPI_MATCH_MODE_AND;
  pollingConfig->AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
}

// status reg SRWD|QE|BP3|BP2|BP1|BP0|WEL|WIP
void set_polling_config_QE(QSPI_AutoPollingTypeDef* pollingConfig)
{
  pollingConfig->Match = 0x40;
  pollingConfig->Mask = 0x40;
  pollingConfig->Interval = 1;
  pollingConfig->StatusBytesSize = 1;
  pollingConfig->MatchMode = QSPI_MATCH_MODE_AND;
  pollingConfig->AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
}

void check_status_register(QSPI_AutoPollingTypeDef* pollingConfig)
{
  QSPI_CommandTypeDef command;
  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  command.Instruction = READ_STATUS_REG;
  command.Address = 0;
  command.AddressSize = 0;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.AlternateBytesSize = 0;
  command.AlternateBytes = 0;
  command.DummyCycles = 0;
  command.DataMode = QSPI_DATA_1_LINE;
  command.NbData = 0;

  if (HAL_QSPI_AutoPolling(&hqspi, &command, pollingConfig, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_QUADSPI_Init();
  MX_USART2_UART_Init();
  
  QSPI_CommandTypeDef command;
  /* USER CODE BEGIN 2 */
  command.AlternateBytesSize = 0;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
  command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  command.DummyCycles = 0;

  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 0; // ClockPrescaler = 0,QSPI clock = FAHB / 1 = 80MHz / 1 = 80MHz
  hqspi.Init.FifoThreshold = 8;  // FIFO when 8 more bytes written or read
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE; // don't sample the data read from memory half-clock cycle later
  hqspi.Init.FlashSize = 25; // flash size = 2**(25+1) = 2**26 = 67108864 = 64 Mbytes
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_3_CYCLE; // the read and wirte command should CS# high in 30ns
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0; // clock stay low bwteen two command
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
