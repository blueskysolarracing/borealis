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
#include "uMPPT_STM32L071RBTx.h"
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
 ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

//----- VARIABLE INIT -----//
//--- ADC VARIABLES ---//
ADC_ChannelConfTypeDef sConfig_ADC;
uint32_t ADC_CH_list[7] = {ADC_CHANNEL_4, ADC_CHANNEL_1, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_9, ADC_CHANNEL_8}; //ADC CH of all 5 uMPPT, output voltage and current

//--- PWM VARIABLES ---//
uint32_t PWM_CHANNEL_list[5] = {TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_4, TIM_CHANNEL_3, TIM_CHANNEL_3};
TIM_HandleTypeDef *PWM_TIM_list[5] = {&htim2, &htim21, &htim3, &htim2, &htim3};

//--- UART VARIABLES ---//
char UART2_rxBuffer[16] = {0};

//Commands set for dev program
char DSBL[6] = "DSBL::";
char NFRE[6] = "NFRE::";
char RFRE[6] = "RFRE::";
char NDUT[6] = "NDUT::";
char RDUT[6] = "RDUT::";
char NPHA[6] = "NPHA::";
char VINP[6] = "VINP::";
char VOUT[6] = "VOUT::";
char IOUT[6] = "IOUT::";
char IINP[6] = "IINP::";
char MPPV[6] = "MPPV::";
char MPPI[6] = "MPPI::";

char uMPPT_ID[1] = {0};
char command[6] = {0};
char value[7] = {0};
char message[16] = {0};
char uMPPT_ID_char[1] = {0};
char ADC_value[7] = {0};

//--- EN PINS VARIABLES ---//
uint32_t EN_Pins_list[5] = {EN1_Pin, EN2_Pin, EN3_Pin, EN4_Pin, EN5_Pin};
GPIO_TypeDef *EN_Ports_list[5] = {GPIOA, GPIOA, GPIOB, GPIOB, GPIOB};

//--- OTHER VARIABLES ---//
uint16_t MCU_OK_LED_PERIOD = 1000; //Blink period of the MCU_OK_LED (ms)
uint16_t MCU_OK_LED_ON_DURATION = 50; //Duration of the ON state of the MCU_OK_LED (ms)

struct uMPPT uMPPT_1;
struct uMPPT uMPPT_2;
struct uMPPT uMPPT_3;
struct uMPPT uMPPT_4;
struct uMPPT uMPPT_5;

struct uMPPT* uMPPT_list[5] = {&uMPPT_1, &uMPPT_2, &uMPPT_3, &uMPPT_4, &uMPPT_5};
struct board_param board;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM21_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
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
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM21_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Configure uMPPT
  board.EN_Pins = EN_Pins_list;
  board.EN_Ports = EN_Ports_list;
  board.MCU_OK_LED_TIM = &htim6;
  board.PWM_CHANNEL = PWM_CHANNEL_list;
  board.PWM_TIM = PWM_TIM_list;
  board.uMPPT_list = uMPPT_list;
  board.MCU_OK_LED_ON_Duration = 100;
  board.MCU_OK_LED_Period = 1000;
  board.ADC_CH = ADC_CH_list;
  board.ADC_config = &sConfig_ADC;
  board.hadc_handle = &hadc;
  board.huart_handle = &huart2;

  config_uMPPT(&board);

//  //Enable LTC7060 gate drivers
//  uint16_t EN_reg_1 = 0;
//  uint16_t EN_reg_2 = 0;
//  for (int i = 0; i < NUM_UMPPT; i++){
//	if (i < 2){	EN_reg_1 |= board.EN_Pins[i];	}
//	else {	EN_reg_2 |= board.EN_Pins[i];	}
//  }
  GPIOA->BSRR |= EN1_Pin | EN2_Pin;
//  GPIOB->BSRR = EN_reg_2;

  //UART (for dev program)
  //HAL_UART_Receive_DMA(&huart2, (uint8_t*) UART2_rxBuffer, 16);
  /* USER CODE END 2 */

  /* Infinite loop */
  htim2.Instance->CCR1 = 1000;
  htim21.Instance->CCR1 = 1000;
  htim2.Instance->ARR = 2000;
  htim21.Instance->ARR = 2000;

  /* USER CODE BEGIN WHILE */

while (1){
//	for (int i = 0; i < NUM_UMPPT; i++){
//		update_MPP_HillClimb(&board, board.uMPPT_list[i]);
//	}
//	float raw_ADC;
//	char print_str[50];

//	for (int i = 0; i < NUM_AVG_CURRENT; i++){
//		raw_ADC += uMPPT_read_ADC(ADC_CH_list[6], &board); //Measure output current
//	}

//	raw_ADC /= NUM_AVG_CURRENT;
//	raw_ADC = (((raw_ADC / 4095.0) * VDDA) - I_MEAS_OFFSET) / I_SENSE_AMP_RATIO / I_SHUNT_VALUE;
//	sprintf(print_str, "Output current: %fA\n", raw_ADC);
//	HAL_UART_Transmit(board.huart_handle, (uint8_t *) print_str, strlen(print_str), 10);

	HAL_Delay(DELAY_BT_MPPT/2);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50000;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 20000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 40000;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim3, TIM3_TI4_GPIOC9_AF2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 26793;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_LED_Pin|EN3_Pin|EN5_Pin|EN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN1_Pin|EN2_Pin|MCU_OK_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC6 PC7 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED_Pin EN3_Pin EN5_Pin EN4_Pin */
  GPIO_InitStruct.Pin = GPIO_LED_Pin|EN3_Pin|EN5_Pin|EN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB3 PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN1_Pin EN2_Pin MCU_OK_LED_Pin */
  GPIO_InitStruct.Pin = EN1_Pin|EN2_Pin|MCU_OK_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FLT5_Pin */
  GPIO_InitStruct.Pin = FLT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLT5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FLT4_Pin FLT3_Pin FLT2_Pin FLT1_Pin */
  GPIO_InitStruct.Pin = FLT4_Pin|FLT3_Pin|FLT2_Pin|FLT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/*! \brief Check which version of the timer triggered this callback and update LED
	 *
	 *  Input: None
	 *  Return: Void
	 */

	if (htim == &htim6){
	    HAL_GPIO_TogglePin(GPIOA, MCU_OK_LED_Pin);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ //Fault input interrupt function
    if(GPIO_Pin == FLT1_Pin || GPIO_Pin == FLT2_Pin || GPIO_Pin == FLT3_Pin || GPIO_Pin == FLT4_Pin || GPIO_Pin == FLT5_Pin){ // If The INT Source Is one of the FLT lines from the LTC7060
    	MCU_OK_LED_PERIOD = 200; //Increase MCU_OK_LED blink frequency to 5Hz to signal that there is a problem
    	htim6.Instance->ARR = 200; //Increase MCU_OK_LED blink frequency to 5Hz to signal that there is a problem

    	//Turn off PWM and disable LTC7060
    	for (int i = 0; i < NUM_UMPPT; i++){
    		HAL_GPIO_WritePin(EN_Ports_list[i], EN_Pins_list[i], GPIO_PIN_RESET);
    		HAL_TIM_PWM_Stop(PWM_TIM_list[i], PWM_CHANNEL_list[i]);
    	}
    }
}

void slice_str(const char * str, uint8_t * buffer, size_t start, size_t end)
{
  size_t j = 0;
  for ( size_t i = start; i <= end; ++i ) {
    buffer[j++] = str[i];
  }
  buffer[j] = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //Echo
	HAL_UART_Transmit(&huart2, UART2_rxBuffer, 16, 100);
	HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
	HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, 16);

	//Extract parameters from command
	slice_str(UART2_rxBuffer, command, 0, 5);
	slice_str(UART2_rxBuffer, uMPPT_ID, 6, 6);
	slice_str(UART2_rxBuffer, value, 9, 15);

	uMPPT_ID[0] = atoi(uMPPT_ID);

	if (strncmp(command, DSBL, 6) == 0){ //DISABLE

	} else if (strncmp(command, NFRE, 6) == 0){ //NEW FREQUENCY
		float new_freq = atof(value);
		updatePWMFreq(uMPPT_list[(int) (uMPPT_ID[0] - 1)], &board, new_freq);

	} else if (strncmp(command, RFRE, 6) == 0){ //READ FREQUENCY
		sprintf(value, "%f", uMPPT_list[(int) (uMPPT_ID[0] - 1)]->pwm_frequency);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = RFRE[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);

	} else if (strncmp(command, NDUT, 6) == 0){ //NEW DUTY CYCLE
		float new_duty_cycle = atof(value);
		updatePWMDutyCycle(uMPPT_list[(int) (uMPPT_ID[0] - 1)], &board, new_duty_cycle);

	} else if (strncmp(command, RDUT, 6) == 0){ //READ DUTY CYCLE
		sprintf(value, "%f", uMPPT_list[(int) (uMPPT_ID[0] - 1)]->pwm_duty_cycle);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = RDUT[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);

	} else if (strncmp(command, NPHA, 6) == 0){ //NEW PWM PHASE
		float new_phase = atof(value);
		updatePWMPhaseOffset(uMPPT_list[(int) (uMPPT_ID[0] - 1)], &board, new_phase/360.0);

	} else if (strncmp(command, VINP, 6) == 0){ //READ INPUT VOLTAGE
		sprintf(ADC_value, "%f", uMPPT_list[(int) (uMPPT_ID[0] - 1)]->input_voltage);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = VINP[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = ADC_value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);

	} else if (strncmp(command, VOUT, 6) == 0){ //READ OUTPUT VOLTAGE
		sprintf(ADC_value, "%f", board.V_Out);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = VOUT[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = ADC_value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);

	} else if (strncmp(command, IOUT, 6) == 0){ //READ OUTPUT CURRENT
		sprintf(ADC_value, "%f", board.I_Out);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = IOUT[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = ADC_value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);

	} else if (strncmp(command, IINP, 6) == 0){ //READ INPUT CURRENT
		sprintf(ADC_value, "%f", uMPPT_list[(int) (uMPPT_ID[0] - 1)]->calc_input_current);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = IINP[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = ADC_value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);

	} else if (strncmp(command, MPPV, 6) == 0){ //READ MPP VOLTAGE
		sprintf(ADC_value, "%f", uMPPT_list[(int) (uMPPT_ID[0] - 1)]->MPP_voltage);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = MPPV[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = ADC_value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);

	} else if (strncmp(command, MPPI, 6) == 0){ //READ MPP CURRENT
		sprintf(ADC_value, "%f", uMPPT_list[(int) (uMPPT_ID[0] - 1)]->MPP_current);
		itoa(uMPPT_ID[0], uMPPT_ID_char, 10);

		//Assembling message
		for(int i = 0; i < 16; i++){
			if (i < 6){message[i] = MPPI[i];}
			else if (i == 6){message[i] = uMPPT_ID_char[0];}
			else if (i == 7 || i == 8){message[i] = ',';}
			else if (i > 7){message[i] = ADC_value[i-9];}}

		HAL_UART_Transmit(&huart2, (uint8_t*) message, 16, 100);
	}
	return;
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
