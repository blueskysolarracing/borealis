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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "newpsm.h"
#include "bmisc.h"
#include "buart.h"
#include "btcp.h"
#include "h7Boot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_INTERVAL 1000 //Period between heartbeats
#define TCP_ID PPTMB_ID
#define CONNECTION_EXPIRY_THRESHOLD 5000 //Number of ticks since last packet received before connection is considered "lost"

enum MPPT {
	AERL_MPPT,
	BSSR_MPPT
};

#define MPPT_DEVICE AERL_MPPT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
B_uartHandle_t *buart;
B_tcpHandle_t *btcp;

uint8_t heartbeat[2] = {PPTMB_HEARTBEAT_ID, 0};

//--- RELAYS & BATTERY ---//
struct relay_periph relay;
QueueHandle_t relayCtrl = NULL;
uint8_t relayCtrlMessage;
uint8_t batteryState = FAULTED; //Assume battery is faulted

//--- PSM ---//
struct PSM_P psmPeriph;
struct PSM_FIR_Filter psmFilter;

float PSM_FIR_voltage[PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB] = {0};
float PSM_FIR_current[PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI2_Init(void);
static void MX_CRC_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void PSMTaskHandler(void * parameters);
void measurementSender(TimerHandle_t xTimer);
void HeartbeatHandler(TimerHandle_t xTimer);
static void arrayRelayTask(void * argument);


//Turn off 12V supply to PPT
void shutDownPPTs() {
	if(MPPT_DEVICE == BSSR_MPPT){
		HAL_GPIO_WritePin(PPT_12V_EN_GPIO_Port, PPT_12V_EN_Pin, GPIO_PIN_SET);
	}
	else if(MPPT_DEVICE == AERL_MPPT){
		HAL_GPIO_WritePin(AERL_PPT_EN_GPIO_Port, AERL_PPT_EN_Pin, GPIO_PIN_RESET);
	}
}

//Turn on 12V supply to PPT
void turnOnPPTs() {
	if(MPPT_DEVICE == BSSR_MPPT){
		HAL_GPIO_WritePin(PPT_12V_EN_GPIO_Port, PPT_12V_EN_Pin, GPIO_PIN_RESET);
	}
	else if(MPPT_DEVICE == AERL_MPPT){
		HAL_GPIO_WritePin(AERL_PPT_EN_GPIO_Port, AERL_PPT_EN_Pin, GPIO_PIN_SET);
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TaskHandle_t PSM_handle;
BaseType_t status;
uint32_t BBMB_last_packet_tick_count = 0;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_CRC_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  //--- NUKE LED ---//
  HAL_TIM_Base_Start_IT(&htim7);

  //--- PPT POWER ---//
  shutDownPPTs(); //Turn off 12V supply to PPT (to prevent gating while the battery isn't connected)

  //--- RDB ---//
  relay.DISCHARGE_GPIO_Port = RELAY_DISCHARGE_GPIO_Port;
  relay.DISCHARGE_Pin = RELAY_DISCHARGE_Pin;
  relay.GND_SIG_GPIO_Port = RELAY_LS_GPIO_Port;
  relay.GND_SIG_Pin = RELAY_LS_Pin;
  relay.ON_SIG_GPIO_Port = RELAY_HS_GPIO_Port;
  relay.ON_SIG_Pin = RELAY_HS_Pin;
  relay.PRE_SIG_GPIO_Port = RELAY_PRECHARGE_GPIO_Port;
  relay.PRE_SIG_Pin = RELAY_PRECHARGE_Pin;
  relay.battery_relay_state = OPEN; //Assume array relays are opened until confirmation is received from PPTMB
  relay.array_relay_state = OPEN; 	//Open array relays at startup

  //--- PSM ---//
  psmPeriph.CSPin = PSM_CS_0_Pin;
  psmPeriph.CSPort = PSM_CS_0_GPIO_Port;
  psmPeriph.LVDSPort = PSM_LVDS_EN_GPIO_Port;
  psmPeriph.LVDSPin = PSM_LVDS_EN_Pin;
  psmPeriph.motherboard = PPTMB_PSM;

  PSM_init(&psmPeriph, &hspi2, &huart2);
  PSM_FIR_Init(&psmFilter);

  test_config(&psmPeriph, &hspi2, &huart2);

  psmFilter.buf_voltage = PSM_FIR_voltage;
  psmFilter.buf_current = PSM_FIR_current;
  psmFilter.buf_size = PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB;
//  test_config(&psmPeriph, &hspi2, &huart2);
//
//  if (config_PSM(&psmPeriph, &hspi2, &huart2, "1234", 2000) == -1){ //2000ms timeout
//	  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //Turn on red LED as a warning


//  }

  //--- COMMS ---//
  buart = B_uartStart(&huart4);
  btcp = B_tcpStart(PPTMB_ID, &buart, buart, 1, &hcrc);

  relayCtrl = xQueueCreate(4, sizeof(uint8_t)); //Holds instruction to open (1) or close relay (2)
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  //--- FREERTOS ---//
  // configASSERT(xTimerStart(xTimerCreate("PSMTaskHandler",  pdMS_TO_TICKS(round(1000 / PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB)), pdTRUE, (void *)0, PSMTaskHandler), 0)); //Temperature and voltage measurements
  configASSERT(xTimerStart(xTimerCreate("measurementSender",  pdMS_TO_TICKS(PSM_SEND_INTERVAL), pdTRUE, (void *)0, measurementSender), 0)); //Periodically send data on UART bus
  xTimerStart(xTimerCreate("HeartbeatHandler",  pdMS_TO_TICKS(HEARTBEAT_INTERVAL / 2), pdTRUE, (void *)0, HeartbeatHandler), 0); //Heartbeat handler

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
#ifdef DEFAULT_TASK

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#endif
  BaseType_t status;
  TaskHandle_t arrayRelayTaskHandle;
  status = xTaskCreate(arrayRelayTask,  /* Function that implements the task. */
			"arrayRelayTask", /* Text name for the task. */
			200, 		/* 200 words *4(bytes/word) = 800 bytes allocated for task's stack*/
			"none", /* Parameter passed into the task. */
			4, /* Priority at which the task is created. */
			&arrayRelayTaskHandle /* Used to pass out the created task's handle. */
						  );
  configASSERT(status);

  TaskHandle_t PSM_handle;
  status = xTaskCreate(PSMTaskHandler,  // Function that implements the task.
						"PSMTask",  // Text name for the task.
						200, 		 // 200 words *4(bytes/word) = 800 bytes allocated for task's stack
						"none",  // Parameter passed into the task.
						4,  // Priority at which the task is created.
						&PSM_handle  // Used to pass out the created task's handle.
									);
  configASSERT(status == pdPASS); // Error checking

  /* USER CODE END RTOS_THREADS */

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
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 31999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart4.Init.BaudRate = 500000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RELAY_HS_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, RELAY_LS_Pin|RELAY_DISCHARGE_Pin|PSM_LVDS_EN_Pin|PSM_CS_1_Pin
                          |PSM_CS_3_Pin|PSM_CS_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PSM_DReady_GPIO_Port, PSM_DReady_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, PSM_CS_2_Pin|AERL_PPT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PPT_12V_EN_GPIO_Port, PPT_12V_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RELAY_HS_Pin */
  GPIO_InitStruct.Pin = RELAY_HS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_HS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_LS_Pin RELAY_DISCHARGE_Pin */
  GPIO_InitStruct.Pin = RELAY_LS_Pin|RELAY_DISCHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_PRECHARGE_Pin */
  GPIO_InitStruct.Pin = RELAY_PRECHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_PRECHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PSM_LVDS_EN_Pin PSM_CS_1_Pin PSM_CS_3_Pin PSM_CS_0_Pin */
  GPIO_InitStruct.Pin = PSM_LVDS_EN_Pin|PSM_CS_1_Pin|PSM_CS_3_Pin|PSM_CS_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PSM_DReady_Pin */
  GPIO_InitStruct.Pin = PSM_DReady_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PSM_DReady_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PSM_CS_2_Pin AERL_PPT_EN_Pin */
  GPIO_InitStruct.Pin = PSM_CS_2_Pin|AERL_PPT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PPT_12V_EN_Pin */
  GPIO_InitStruct.Pin = PPT_12V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PPT_12V_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void PSMTaskHandler(void * parameters){

	double voltage, current;

	int delay = pdMS_TO_TICKS(round(1000 / PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB));

	while (1){

		voltage = readPSM(&psmPeriph, VBUS, 3);
		current = readPSM(&psmPeriph, CURRENT, 3);

		vTaskSuspendAll();

		psmFilter.push(&psmFilter, (float) voltage, VOLTAGE_MEASUREMENT);
		psmFilter.push(&psmFilter, (float) current, CURRENT_MEASUREMENT);

		xTaskResumeAll();
		vTaskDelay(delay);
	}
}

void measurementSender(TimerHandle_t xTimer){
	uint8_t busMetrics_HV[3 * 4] = {0};
//	uint8_t busMetrics_PPT[7 * 4] = {0};

	busMetrics_HV[0] = PPTMB_BUS_METRICS_ID;
	//busMetrics_PPT[0] = PPTMB_PPT_METRICS_ID;

	vTaskSuspendAll();

	float HV_voltage = psmFilter.get_average(&psmFilter, VOLTAGE_MEASUREMENT);
	float HV_current = psmFilter.get_average(&psmFilter, CURRENT_MEASUREMENT);

	xTaskResumeAll();

	floatToArray(HV_voltage, busMetrics_HV + 4); // fills 4 - 7 of busMetrics
	floatToArray(HV_current, busMetrics_HV + 8); // files 8 - 11 of busMetrics

	B_tcpSend(btcp, busMetrics_HV, sizeof(busMetrics_HV));
}

//void PSMTaskHandler(TimerHandle_t xTimer){
////Output of MPPTs
//	double voltageCurrent_total[2] = {0};
//	uint8_t busMetrics_total[3 * 4] = {0};
//	busMetrics_total[0] = PPTMB_BUS_METRICS_ID;
//
//	//PSMRead will fill first element with voltage, second with current
//	PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 2, voltageCurrent_total, 2); //Array output on channel #2
//	floatToArray((float) voltageCurrent_total[0], busMetrics_total + 4); // fills 4 - 7 of busMetrics
//	floatToArray((float) voltageCurrent_total[1], busMetrics_total + 8); // fills 8 - 11 of busMetrics
//
//	B_tcpSend(btcp, busMetrics_total, sizeof(busMetrics_total));
//
////Output of string
//	double voltageCurrent_string[2] = {0};
//	uint8_t busMetrics_string[7 * 4] = {0};
//	busMetrics_string[0] = PPTMB_PPT_METRICS_ID;
//
//	//String #1
//	PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 1, voltageCurrent_string, 2);
//	floatToArray((float) voltageCurrent_string[0], busMetrics_string + 4); // fills 4 - 7 of busMetrics
//	floatToArray((float) voltageCurrent_string[1], busMetrics_string + 16); // fills 16 - 19 of busMetrics
//
//	//String #2
//	PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 3, voltageCurrent_string, 2);
//	floatToArray((float) voltageCurrent_string[0], busMetrics_string + 8); // fills 8 - 11 of busMetrics
//	floatToArray((float) voltageCurrent_string[1], busMetrics_string + 20); // fills 20 - 23 of busMetrics
//
//	//String #3
//	PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 4, voltageCurrent_string, 2);
//	floatToArray((float) voltageCurrent_string[0], busMetrics_string + 12); // fills 11 - 15 of busMetrics
//	floatToArray((float) voltageCurrent_string[1], busMetrics_string + 24); // fills 24 - 27 of busMetrics
//
//	B_tcpSend(btcp, busMetrics_string, sizeof(busMetrics_string));
//}

void HeartbeatHandler(TimerHandle_t xTimer){
	//Send periodic heartbeat so we know the board is still running
	B_tcpSend(btcp, heartbeat, sizeof(heartbeat));
	heartbeat[1] = ~heartbeat[1]; //Toggle for next time
}

void serialParse(B_tcpPacket_t *pkt){
	switch(pkt->senderID){
		case BBMB_ID:
			BBMB_last_packet_tick_count = xTaskGetTickCount();
			//Relay state
			if (pkt->data[0] == BBMB_RELAYS_STATE_ID){

				vTaskSuspendAll();
				if ((pkt->data[3] == OPEN) && (relay.array_relay_state == CLOSED)){ //Open relays and resend
					relayCtrlMessage = RELAY_QUEUE_OPEN_ARRAY;
					xTaskResumeAll();
					xQueueSend(relayCtrl, &relayCtrlMessage, 10); //Open relays
					vTaskSuspendAll();
				} else if ((pkt->data[3] == CLOSED) && (relay.array_relay_state == OPEN)){ //Try to close array relays
					relayCtrlMessage = RELAY_QUEUE_CLOSE_ARRAY;
					xTaskResumeAll();
					xQueueSend(relayCtrl, &relayCtrlMessage, 10); //Close relays
					vTaskSuspendAll();
				}


				batteryState = pkt->data[1];
				relay.battery_relay_state = pkt->data[2];

				//Disable 12V power to PPT if the battery is disconnected from HV bus (to prevent gating without output biased by anything)
				//Note that PPTMB design has PPT_12V_EN GPIO signal on Nuke pin 50. However, no signal is routed there so we shorted that pin to pin 48 on PPTMB-specific Nucleo connector.
				if ((batteryState == FAULTED) || (relay.battery_relay_state == OPEN)) shutDownPPTs();
				//If the battery is OK and both sets of relays are closed, give 12V to PPTs, which starts operation
				else if ((relay.array_relay_state == CLOSED) && (relay.battery_relay_state == CLOSED) && (batteryState == HEALTHY)) turnOnPPTs();
				xTaskResumeAll();
			}
			break;
	}

}

void arrayRelayTask(void * argument){
	uint8_t buf_relay[10];

	//When relays need to be opened, put a 1 in the queue. When they need to be closed, put a 2.

	for(;;){
		if (xQueueReceive(relayCtrl, &buf_relay, 50)){
			if (buf_relay[0] == RELAY_QUEUE_OPEN_ARRAY && relay.array_relay_state != OPEN){
				//For opening relays, notify everyone as soon as the command is received
				relay.array_relay_state = OPEN;

				//Turn off power to MPPTs, which stops operation
				shutDownPPTs();
				osDelay(500); //Wait for 12V-12V converter to turn off

				//Send new relay state to bus
				uint8_t buf[8] = {PPTMB_RELAYS_STATE_ID, batteryState, relay.battery_relay_state, relay.array_relay_state,
								  4, 0, 0, 0}; //Send fault type of "4" which doesn't correspond to any fault
				B_tcpSend(btcp, buf, sizeof(buf));

				open_relays(&relay);

			} else if (buf_relay[0] == RELAY_QUEUE_CLOSE_ARRAY && relay.array_relay_state != CLOSED){
				close_relays(&relay);

				//Only after the relays are fully closed do you update the bus and internal variables
				relay.array_relay_state = CLOSED;

				uint8_t buf[8] = {PPTMB_RELAYS_STATE_ID, batteryState, relay.battery_relay_state, relay.array_relay_state,
								  0, 0, 0, 0};
				B_tcpSend(btcp, buf, sizeof(buf));

				//If the battery is OK and both sets of relays are closed, give 12V to PPTs, which starts operation
				if ((relay.array_relay_state == CLOSED) && (relay.battery_relay_state == CLOSED) && (batteryState == HEALTHY)) turnOnPPTs();
			}
		} else {// Nothing is retrieved from queue after waiting

			uint32_t tick_cnt = xTaskGetTickCount();

			// Automatically open array relays and shut down PPTs if no message has been heard from BBMB after CONNECTION_EXPIRY_THRESHOLD
			if (((tick_cnt - BBMB_last_packet_tick_count) > CONNECTION_EXPIRY_THRESHOLD) && (relay.array_relay_state != OPEN)){


				//For opening relays, notify everyone as soon as possible
				relay.array_relay_state = OPEN;

				//Turn off power to MPPTs, which stops operation
				shutDownPPTs();
				osDelay(500); //Wait for 12V-12V converter to turn off

				//Send new relay state to bus
				uint8_t buf[8] = {PPTMB_RELAYS_STATE_ID, batteryState, relay.battery_relay_state, relay.array_relay_state,
								  4, 0, 0, 0}; //Send fault type of "4" which doesn't correspond to any fault
				B_tcpSend(btcp, buf, sizeof(buf));

				open_relays(&relay);
			}
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  osThreadTerminate(defaultTaskHandle);
  /* USER CODE END 5 */
}

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
  else if (htim == &htim7){
  	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  }
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
