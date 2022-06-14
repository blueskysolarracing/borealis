/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BSSR_SERIAL_START 0xa5
#define BSSR_SPB_SWB_ACK 0x77 //Acknowledge signal sent back from DCMB upon reception of data from SPB/SWB (77 is BSSR team number :D)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t getSwitchState(){
	uint8_t switchState = 0;

	switchState |= HAL_GPIO_ReadPin(ARRAY_GPIO_Port, ARRAY_Pin) << 0;
	switchState |= HAL_GPIO_ReadPin(GPIOC, AUX0_Pin) 			<< 1;
	switchState |= HAL_GPIO_ReadPin(GPIOB, AUX1_Pin) 			<< 2;
	switchState |= HAL_GPIO_ReadPin(GPIOB, AUX2_Pin) 			<< 3;
	switchState |= HAL_GPIO_ReadPin(GPIOC, FAN_Pin) 			<< 4;
	switchState |= HAL_GPIO_ReadPin(GPIOB, FWD_REV_Pin) 		<< 5;
	switchState |= HAL_GPIO_ReadPin(GPIOB, CAMERA_Pin) 			<< 6;
	switchState |= HAL_GPIO_ReadPin(GPIOC, IGNITION_Pin) 		<< 7;

	return switchState;
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
  MX_CRC_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  uint8_t oldSwitchState = 0;
  uint8_t newSwitchState = getSwitchState();

  //Data is collected as a byte with the following formatting: [IGNITION, CAMERA, FWD/REV, FAN, AUX2, AUX1, AUX0, ARRAY] ([bit 7...bit 0])
  //Data is sent: 1) When board first starts, 2) on any change in switch state

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 newSwitchState = getSwitchState();
	 uint8_t buf[4] = {BSSR_SERIAL_START, 0x04, newSwitchState, 0x00}; //Last byte is CRC, optional

	  if (newSwitchState != oldSwitchState){ //Switches changed; need to send
		  //print_switches();
		  uint8_t rx_buf[2];
		  do { //Keep sending data until acknowledge is received from DCMB
			  HAL_UART_Transmit(&huart2, sizeof(buf), 4, 10); //To DCMB
			  HAL_UART_Transmit(&huart4, sizeof(buf), 4, 10); //To debug
			  HAL_UART_Receive(&huart2, sizeof(rx_buf), 2, 100);
			  HAL_Delay(1);
		  } while (rx_buf[1] != BSSR_SPB_SWB_ACK);
	  }

	  oldSwitchState = newSwitchState;

	  HAL_Delay(5); //Wait for 5ms; could be replaced with power down sleep
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
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
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : FAN_Pin IGNITION_Pin AUX0_Pin */
  GPIO_InitStruct.Pin = FAN_Pin|IGNITION_Pin|AUX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARRAY_Pin */
  GPIO_InitStruct.Pin = ARRAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARRAY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AUX1_Pin FWD_REV_Pin AUX2_Pin CAMERA_Pin */
  GPIO_InitStruct.Pin = AUX1_Pin|FWD_REV_Pin|AUX2_Pin|CAMERA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void print_switches(){
  if (HAL_GPIO_ReadPin(GPIOC, FAN_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "FAN: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "FAN: 1\n", 30, 100);	}

  if (HAL_GPIO_ReadPin(GPIOC, AUX0_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "AUX0: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "AUX0: 1\n", 30, 100);	}

  if (HAL_GPIO_ReadPin(GPIOB, AUX1_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "AUX1: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "AUX1: 1\n", 30, 100);	}

  if (HAL_GPIO_ReadPin(GPIOB, AUX2_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "AUX2: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "AUX2: 1\n", 30, 100);	}

  if (HAL_GPIO_ReadPin(ARRAY_GPIO_Port, ARRAY_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "ARRAY: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "ARRAY: 1\n", 30, 100);	}

  if (HAL_GPIO_ReadPin(GPIOB, FWD_REV_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "FWD/REV: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "FWD/REV: 1\n", 30, 100);	}

  if (HAL_GPIO_ReadPin(GPIOB, CAMERA_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "CAMERA: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "CAMERA: 1\n", 30, 100);	}

  if (HAL_GPIO_ReadPin(GPIOC, IGNITION_Pin)){	HAL_UART_Transmit(&huart4, (uint8_t*) "IGNITION: 0\n", 30, 100);	}
  else {	HAL_UART_Transmit(&huart4, (uint8_t*) "IGNITION: 1\n", 30, 100);	}

  HAL_UART_Transmit(&huart4, (uint8_t*) "\n\n", 5, 100);
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
  if (htim->Instance == TIM6) {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

