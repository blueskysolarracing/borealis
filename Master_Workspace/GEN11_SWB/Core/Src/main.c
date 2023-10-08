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
#define ADC_NUM_AVG 30.0 // Number of times to loop to get average ADC value
#define REGEN_IDLE_VAL 52.0
#define REGEN_MAX_VAL 115.0
#define REGEN_NEW_MAX 255
#define REGEN_OFFSET  19
#define REGEN_MULTIPLIER -63.75
//#define USE_ADC_REGEN
#define USE_ACC_ENC

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t counter = 0;
uint8_t bstate = 0;
uint8_t astate = 0;
static int outputVal = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
//static void switchStateTask(void const* pv);

uint8_t getSwitchState(uint8_t whichByte){
	uint8_t switchState[3] = {0, 0, 0};

	//-- BYTE 0 = [ACC8, ACC7, ACC6, ACC5, ACC4, ACC3, ACC2, ACC1] --//
	//Accelerator rotary encoder
	switch (whichByte){
	case 0:
		//switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC1_Pin) << 0;
		//switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC2_Pin) << 1;
		switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC3_Pin) << 2;
		switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC4_Pin) << 3;
		switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC5_Pin) << 4;
		switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC6_Pin) << 5;
		switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC7_Pin) << 6;
		switchState[0] |= HAL_GPIO_ReadPin(GPIOC, ACC8_Pin) << 7;

		//	switchState[0] = (switchState[0] & ~0b00000001) | (HAL_GPIO_ReadPin(GPIOC, ACC1_Pin) & 0b00000001); //Bit 0 - OK
		//	switchState[0] = (switchState[0] & ~0b00000010) | ((HAL_GPIO_ReadPin(GPIOC, ACC2_Pin) << 1) & 0b00000010); //Bit 1 - OK
		//	switchState[0] = (switchState[0] & ~0b00000100) | ((HAL_GPIO_ReadPin(GPIOC, ACC3_Pin) << 2) & 0b00000100); //Bit 2 - OK
		//	switchState[0] = (switchState[0] & ~0b00001000) | ((HAL_GPIO_ReadPin(GPIOC, ACC4_Pin) << 3) & 0b00001000); //Bit 3 - OK
		//	switchState[0] = (switchState[0] & ~0b00010000) | ((HAL_GPIO_ReadPin(GPIOC, ACC5_Pin) << 4) & 0b00010000); //Bit 4 - OK
		//	switchState[0] = (switchState[0] & ~0b00100000) | ((HAL_GPIO_ReadPin(GPIOC, ACC6_Pin) << 5) & 0b00100000); //Bit 5 - OK
		//	switchState[0] = (switchState[0] & ~0b01000000) | ((HAL_GPIO_ReadPin(GPIOC, ACC7_Pin) << 6) & 0b01000000); //Bit 6 - OK
		//	switchState[0] = (switchState[0] & ~0b10000000) | ((HAL_GPIO_ReadPin(GPIOC, ACC8_Pin) << 7) & 0b10000000); //Bit 7 - OK
		switchState[0] = ~switchState[0];
		return switchState[0];

	case 1:
		//-- BYTE 1 = [x, x, x, CRUISE_SIGNAL_Pin, HORN_SIGNAL_Pin, RAD_SIGNAL_Pin, R_SIGNAL_Pin, L_SIGNAL_Pin] --//
		//Left/right turn signals
		switchState[1] |= HAL_GPIO_ReadPin(GPIOC, L_SIGNAL_Pin) << 0;
		switchState[1] |= HAL_GPIO_ReadPin(GPIOC, R_SIGNAL_Pin) << 1;

	//	switchState[1] = (switchState[1] & ~0b00000001) | (HAL_GPIO_ReadPin(GPIOC, L_SIGNAL_Pin) & 0b00000001); //Bit 0 - OK
	//	switchState[1] = (switchState[1] & ~0b00000010) | ((HAL_GPIO_ReadPin(GPIOC, R_SIGNAL_Pin) << 1) & 0b00000010); //Bit 1 - OK

		//Radio
		switchState[1] |= HAL_GPIO_ReadPin(GPIOC, RAD_SIGNAL_Pin) << 2;
	//	switchState[1] = (switchState[1] & ~0b00000100) | ((HAL_GPIO_ReadPin(GPIOC, RAD_SIGNAL_Pin) << 2) & 0b00000100); //Bit 2 - OK

		//Horn
		switchState[1] |= HAL_GPIO_ReadPin(GPIOC, HORN_SIGNAL_Pin) << 3;
	//	switchState[1] = (switchState[1] & ~0b00001000) | ((HAL_GPIO_ReadPin(GPIOC, HORN_SIGNAL_Pin) << 3) & 0b00001000); //Bit 3 - OK

		//Cruise control
		switchState[1] |= HAL_GPIO_ReadPin(GPIOB, CRUISE_SIGNAL_Pin) << 4;
	//	switchState[1] = (switchState[1] & ~0b00010000) | ((HAL_GPIO_ReadPin(GPIOB, CRUISE_SIGNAL_Pin) << 4) & 0b00010000); //Bit 4 - OK

		switchState[1] = ~switchState[1] & 0b00011111; //Make sure other bits are 0
		return switchState[1];

	case 2:
		//-- BYTE 1 = [x, x, x, SELECT_Pin, RIGHT_Pin, LEFT_Pin, DOWN_Pin, UP_Pin] --//
		//Navigation
		switchState[2] |= HAL_GPIO_ReadPin(GPIOB, UP_Pin) << 0;
		switchState[2] |= HAL_GPIO_ReadPin(GPIOB, DOWN_Pin) << 1;
		switchState[2] |= HAL_GPIO_ReadPin(GPIOB, LEFT_Pin) << 2;
		switchState[2] |= HAL_GPIO_ReadPin(GPIOB, RIGHT_Pin) << 3;
		switchState[2] |= HAL_GPIO_ReadPin(GPIOB, SELECT_Pin) << 4;

	//	switchState[2] = (switchState[2] & ~0b00000001) | (HAL_GPIO_ReadPin(GPIOB, UP_Pin) & 0b00000001); //Bit 0 - OK
	//	switchState[2] = (switchState[2] & ~0b00000010) | ((HAL_GPIO_ReadPin(GPIOB, DOWN_Pin) << 1) & 0b00000010); //Bit 1 - OK
	//	switchState[2] = (switchState[2] & ~0b00000100) | ((HAL_GPIO_ReadPin(GPIOB, LEFT_Pin) << 2) & 0b00000100); //Bit 2 - OK
	//	switchState[2] = (switchState[2] & ~0b00001000) | ((HAL_GPIO_ReadPin(GPIOB, RIGHT_Pin) << 3) & 0b00001000); //Bit 3 - OK
	//	switchState[2] = (switchState[2] & ~0b00010000) | ((HAL_GPIO_ReadPin(GPIOB, SELECT_Pin) << 4) & 0b00010000); //Bit 4 - OK

		switchState[2] = ~switchState[2] & 0b00011111; //Make sure other bits are 0
		return switchState[2];
	}
}
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
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t oldSwitchState[3] = {0, 0, 0};
  uint8_t newSwitchState[3] = {0, 0, 0};
  uint8_t oldRegenValue = 0;
  uint8_t oldAccValue = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//newSwitchState[0] = getSwitchState(0);
	newSwitchState[1] = getSwitchState(1);
	newSwitchState[2] = getSwitchState(2);
	uint8_t newRegenValue = 0;
	uint8_t newAccValue = 0;

	float regenTotalReading = 0;

	#ifdef USE_ADC_REGEN
		for (int i = 0; i < ADC_NUM_AVG; i++){
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
				float currRegenValue = HAL_ADC_GetValue(&hadc1);
				regenTotalReading += currRegenValue;
			}
		}

		newRegenValue = ((uint8_t)(regenTotalReading/ADC_NUM_AVG) - REGEN_OFFSET) * (REGEN_MULTIPLIER);
	#endif

	#ifdef USE_ACC_ENC
		if (counter>255){
			newAccValue = 255;
		}
		else if (counter<0){
			newAccValue = 0;
		}
		else{
			newAccValue = counter;
		}

	#endif

	if ((oldSwitchState[1] != newSwitchState[1]) || (oldSwitchState[2] != newSwitchState[2]) || (oldRegenValue != newRegenValue) || (oldAccValue != newAccValue)){ //If any bit has changed, send data

		//Map regen range from 52-115 to 0-255
		uint8_t mappedRegenValue = 0;

		#ifdef USE_ADC_REGEN
				if (newRegenValue > REGEN_IDLE_VAL){
					if (newRegenValue >= REGEN_MAX_VAL){
						mappedRegenValue = REGEN_NEW_MAX;
					} else {
						float mappingRatio = (newRegenValue - REGEN_IDLE_VAL)/(REGEN_MAX_VAL - REGEN_IDLE_VAL);
						mappedRegenValue = (uint8_t)(mappingRatio * REGEN_NEW_MAX);
					}
				}
		#endif


		#ifdef USE_ACC_ENC
			static uint8_t started = 0;
			static int currentValue = 0;
			int positiveTurn = 0;
			int negativeTurn = 0;
			int difference = 0;
			//static int outputVal = 0;
			int accValTemp = newAccValue == 255 ? currentValue : newAccValue;
			if(!started){
				started++;
				currentValue = accValTemp;
			} else{
				positiveTurn = (currentValue + 63) % 128;
				negativeTurn = (currentValue - 64) % 128;
				if(positiveTurn > currentValue){
					if(accValTemp <= positiveTurn && accValTemp >= currentValue){
						difference = accValTemp - currentValue;
					} else {
						if(accValTemp < currentValue){
							difference = accValTemp - currentValue;
						} else {
							difference = -(128 -(accValTemp - currentValue));
						}
					}
				} else {
					if(accValTemp <= currentValue && accValTemp >= negativeTurn){
						difference = -(currentValue - accValTemp);
					} else {
						if(currentValue < accValTemp){
							difference = accValTemp - currentValue;
						} else {
							difference = 128 - (currentValue - accValTemp);
						}
					}
				}
				difference = difference < 0 ? -difference * difference : difference * difference;
				outputVal += difference;
		//		sprintf(buf2, "c=%d,w=%d,d=%d,o=%d\r\n", currentValue, accValTemp, difference, outputVal);
				currentValue = accValTemp;
				if(outputVal < 0){
					outputVal = 0;
				} else if (outputVal > 255){
					outputVal = 255;
				}
			}
		#endif

		uint8_t buf[7] = {BSSR_SERIAL_START, 0x03, newSwitchState[0], newSwitchState[1], newSwitchState[2], mappedRegenValue, outputVal}; // last byte could be used for CRC (optional)
		uint8_t rx_buf[2];

		//do { //Keep sending data until acknowledge is received from DCMB
			HAL_UART_Transmit(&huart2, buf, sizeof(buf), 100); //To DCMB
			HAL_UART_Transmit(&huart4, buf, sizeof(buf), 100); //To debug
//			HAL_UART_Receive(&huart2, rx_buf, sizeof(rx_buf), 100);
			HAL_Delay(1);
			// Test ==============
//			char ary[20] = "hello world";
//			HAL_UART_Transmit(&huart2, ary, strlen(ary), 1000);
		//} while (rx_buf[1] != BSSR_SPB_SWB_ACK);
	}

	//Update switch state
	for (int i = 0; i < 3; i++){ oldSwitchState[i] = newSwitchState[i];}
	oldRegenValue = newRegenValue;
	oldAccValue = newAccValue;

	//HAL_Delay(15); //Wait for 15ms; could be replaced with power down sleep
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : ACC8_Pin R_SIGNAL_Pin L_SIGNAL_Pin RAD_SIGNAL_Pin
                           HORN_SIGNAL_Pin ACC3_Pin ACC4_Pin ACC5_Pin
                           ACC6_Pin ACC7_Pin */
  GPIO_InitStruct.Pin = ACC8_Pin|R_SIGNAL_Pin|L_SIGNAL_Pin|RAD_SIGNAL_Pin
                          |HORN_SIGNAL_Pin|ACC3_Pin|ACC4_Pin|ACC5_Pin
                          |ACC6_Pin|ACC7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 PA8
                           PA9 PA10 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB3 PB4 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : UP_Pin LEFT_Pin DOWN_Pin RIGHT_Pin
                           SELECT_Pin CRUISE_SIGNAL_Pin */
  GPIO_InitStruct.Pin = UP_Pin|LEFT_Pin|DOWN_Pin|RIGHT_Pin
                          |SELECT_Pin|CRUISE_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
//static void switchStateTask(void const* pv){
//	GPIO_TypeDef* swPort1 = GPIOB; // SELECT, RIGHT, DOWN, LEFT, UP, CRUISE
//	GPIO_TypeDef* swPort2 = (GPIOC & 0xF); // HORN_SIGNAL, RAD_SIGNAL, L_SIGNAL, R_SIGNAL
//	GPIO_TypeDef* swPort3 = (GPIOC >> 4);  // ACC8, ACC7, ACC6, ACC5, ACC4, ACC3, ACC2, ACC1
//
//	#define uartFrame_SIZE 9
//	uint8_t uartFrame[uartFrame_SIZE] = {0xa5, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//
//	for(;;){
//		if(uartFrame[2] == ((swPort1->IDR >> 3) & 0xff) && uartFrame[3] == ((swPort2->IDR >> 10) & 0xff) && uartFrame[4] == ((swPort3->IDR >> 2) & 0xff)) continue;
//		uartFrame[2] = (swPort1->IDR) & 0xff;
//		uartFrame[3] = (swPort2->IDR) & 0xff;
//		uartFrame[4] = (swPort3->IDR) & 0xff;
//
//		uint32_t crc = ~HAL_CRC_Calculate(&hcrc, (uint32_t*) uartFrame, uartFrame_SIZE-4);
//		for(size_t i=0; i<4; i++) uartFrame[i+5] = (crc >> (i << 3)) & 0xff;
//		HAL_UART_Transmit_IT(&huart4, uartFrame, uartFrame_SIZE);
//		HAL_UART_Transmit_IT(&huart2, uartFrame, uartFrame_SIZE);
//		osDelay(1000);
//	}
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_7){
		if (bstate == 0){
			counter++;
		}
		else{
			bstate = 0;
		}
	}
	if(GPIO_Pin == GPIO_PIN_6){
		counter--;
		bstate = 1;

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

