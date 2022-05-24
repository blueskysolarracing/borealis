/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "psm.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

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

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //PSM-related variables
	struct PSM_Peripheral psmPeriph;

	psmPeriph.CSPin0 = SPI2_CS_0_Pin;
	psmPeriph.CSPin1 = SPI2_CS_1_Pin;
	psmPeriph.CSPin2 = SPI2_CS_2_Pin;
	psmPeriph.CSPin3 = SPI2_CS_3_Pin;

	psmPeriph.CSPort0 = SPI2_CS_0_GPIO_Port;
	psmPeriph.CSPort1 = SPI2_CS_1_GPIO_Port;
	psmPeriph.CSPort2 = SPI2_CS_2_GPIO_Port;
	psmPeriph.CSPort3 = SPI2_CS_3_GPIO_Port;

	psmPeriph.LVDSPort = LVDS_EN_GPIO_Port;
	psmPeriph.LVDSPin = LVDS_EN_Pin;

	psmPeriph.DreadyPin = DReady_Pin;
	psmPeriph.DreadyPort = DReady_GPIO_Port;

	PSM_Init(&psmPeriph, 3); //2nd argument is PSM ID
	configPSM(&psmPeriph, &hspi2, &huart2, "1234", 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  char printString[50];

	  double PSMBuffer[30] = {-1, -1, -1};
	  PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 4, PSMBuffer, 2);
	  sprintf(printString, "PSMBuffer[0]: %lf\nPSMBuffer[1]: %lf\n", PSMBuffer[0], PSMBuffer[1]);
	  HAL_UART_Transmit(&huart2, (uint8_t*) printString, strlen(printString), 10);

//	  double PSMBuffer2[30] = {-1, -1, -1};
//	  PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 1, PSMBuffer2, 2);
//	  sprintf(printString, "PSMBuffer2[0]: %lf\nPSMBuffer2[1]: %lf\n\n", PSMBuffer2[0], PSMBuffer2[1]);
//	  HAL_UART_Transmit(&huart2, (uint8_t*) printString, strlen(printString), 10);

	  HAL_Delay(1000);

//	  int receiveBuffer = 0;

//	  do {
//		  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//		  HAL_SPI_Transmit(&hspi2, (uint8_t *) 0x9, 1, 10);
//		  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
//		  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//		  HAL_SPI_Receive(&hspi2, receiveBuffer, 1, 10);
//		  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
//		  HAL_Delay(100);
//	  } while ((receiveBuffer & 0b00000001) != 0);

	  //Wait for DREADY to be low

//	  int dataBuffer[30] = {0, 0, 0};
//	  char printString[50];
//
//	  //CURRENT
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi2, (uint8_t *) 0x0, 1, 10);
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//	  HAL_SPI_Receive(&hspi2, &dataBuffer, 1, 10);
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
////	  for (int i = 0; i < 3; i++){
////		  sprinft(printString, "Current[%d]: %d\n", i, dataBuffer[i]);
////		  HAL_UART_Transmit(&huart2, "Data not ready\n", strlen(printString), 10);
////	  }
//
//	  //VOLTAGE 1
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi2, (uint8_t *) 0x1, 1, 10);
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//	  HAL_SPI_Receive(&hspi2, &dataBuffer, 1, 10);
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
////	  for (int i = 0; i < 3; i++){
////		  sprinft(printString, "Voltage1[%d]: %d\n", i, dataBuffer[i]);
////		  HAL_UART_Transmit(&huart2, "Data not ready\n", strlen(printString), 10);
////	  }
//
//	  //VOLTAGE 2
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi2, (uint8_t *) 0x2, 1, 10);
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_RESET);
//	  HAL_SPI_Receive(&hspi2, &dataBuffer, 1, 10);
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//
////	  for (int i = 0; i < 3; i++){
////		  sprinft(printString, "Voltage2[%d]: %d\n", i, dataBuffer[i]);
////		  HAL_UART_Transmit(&huart2, "Data not ready\n", strlen(printString), 10);
////	  }
//
//	  HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, GPIO_PIN_SET);
//	  HAL_Delay(250);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LVDS_EN_GPIO_Port, LVDS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SPI2_CS_1_Pin|SPI2_CS_2_Pin|SPI2_CS_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_0_GPIO_Port, SPI2_CS_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PI8 PI9 PI10 PI11
                           PI12 PI13 PI14 PI15
                           PI4 PI5 PI6 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC6 PC7 PC8
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF7
                           PF8 PF9 PF10 PF11
                           PF12 PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 PH2 PH3
                           PH4 PH5 PH6 PH7
                           PH8 PH9 PH10 PH11
                           PH12 PH13 PH14 PH15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 PA8 PA9
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB14 PB15
                           PB3 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PJ0 PJ1 PJ2 PJ3
                           PJ4 PJ5 PJ6 PJ7
                           PJ8 PJ9 PJ10 PJ11
                           PJ12 PJ13 PJ14 PJ15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG5 PG6
                           PG7 PG8 PG9 PG10
                           PG11 PG12 PG13 PG14
                           PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LVDS_EN_Pin */
  GPIO_InitStruct.Pin = LVDS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LVDS_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PK0 PK1 PK3 PK4
                           PK5 PK6 PK7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : DReady_Pin */
  GPIO_InitStruct.Pin = DReady_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DReady_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_1_Pin SPI2_CS_2_Pin SPI2_CS_3_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_1_Pin|SPI2_CS_2_Pin|SPI2_CS_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_0_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_0_GPIO_Port, &GPIO_InitStruct);

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

