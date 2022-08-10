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
#include "buart.h"
#include "btcp.h"
#include "protocol_ids.h"
#include "bmisc.h"
#include "psm.h"
#include "h7Boot.h"
#include "TMC5160_driver.h"
#include "lights.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HV_BATT_OVERCURRENT_DISCHARGE 10.0 //Overcurrent threshold on HV battery (discharge; A)
#define NUM_BATT_CELLS 29 //Number of series parallel groups in battery pack
#define NUM_BATT_TEMP_SENSORS 3 * 6 //Number of temperature sensors in battery pack
#define BMS_READ_INTERVAL 1000//(Other intervals defined in psm.h and btcp.h)
#define BMS_FLT_CHECK_INTERVAL 10 //Interval at which to read the BMS_FLT pin
#define PROTECTION_ENABLE 0 //Flag to enable (1) or disable (0) relay control
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */

//--- COMMS ---//
B_tcpHandle_t btcp_bms_actual;
B_tcpHandle_t btcp_main_actual;

B_uartHandle_t* buart_main;
B_uartHandle_t* buart_bms;
B_tcpHandle_t* btcp_main = &btcp_main_actual;
B_tcpHandle_t* btcp_bms = &btcp_bms_actual;
uint8_t heartbeat[2] = {BBMB_HEARTBEAT_ID, 0};

//--- LIGHTS ---//
struct lights_stepper_ctrl lightsPeriph;
uint8_t lightInstruction = 0;
QueueHandle_t lightsCtrl = NULL;

//--- PSM ---//
struct PSM_Peripheral psmPeriph;
float battery_current;

//--- RELAYS ---//
struct relay_periph relay;
QueueHandle_t relayCtrl = NULL;

//--- BMS ---//
uint8_t BMS_requesting_from = 7; //Holds which BMS we are requesting data from (needs initial value > 6)
uint8_t BMS_data_received[3] = {0, 0, 0}; //Holds whether we received voltage, temperature and SoC

//--- Battery ---/
uint8_t batteryState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI5_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART4_Init(void);
static void MX_UART8_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void lightsTask(void * argument);
static void relayTask(void * argument);
void PSMTaskHandler(TimerHandle_t xTimer);
void HeartbeatHandler(TimerHandle_t xTimer);
void BMSPeriodicReadHandler(TimerHandle_t xTimer);
void BMS_Flt_Check(TimerHandle_t xTimer);

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
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
  //--- MCU OK LED ---//
  NVIC_EnableIRQ(TIM7_IRQn);
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef*) &htim7); //Blink LED to show that CPU is still alive

  //--- PSM ---//
  psmPeriph.CSPin0 = PSM_CS_0_Pin;
  psmPeriph.CSPin1 = PSM_CS_1_Pin;
  psmPeriph.CSPin2 = PSM_CS_2_Pin;
  psmPeriph.CSPin3 = PSM_CS_3_Pin;

  psmPeriph.CSPort0 = PSM_CS_0_GPIO_Port;
  psmPeriph.CSPort1 = PSM_CS_1_GPIO_Port;
  psmPeriph.CSPort2 = PSM_CS_2_GPIO_Port;
  psmPeriph.CSPort3 = PSM_CS_3_GPIO_Port;

  psmPeriph.LVDSPort = PSM_LVDS_EN_GPIO_Port;
  psmPeriph.LVDSPin = PSM_LVDS_EN_Pin;

  psmPeriph.DreadyPin = PSM_DReady_Pin;
  psmPeriph.DreadyPort = PSM_DReady_GPIO_Port;
  //HAL_TIM_PWM_Start_PWM(&htim7, 5);
  PSM_Init(&psmPeriph, 1); //2nd argument is PSM ID
  if (configPSM(&psmPeriph, &hspi2, &huart2, "12", 2000) == -1){ //2000ms timeout
	  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //Turn on red LED as a warning
  }

  //--- RDB ---//
  relay.DISCHARGE_GPIO_Port = RELAY_DISCHARGE_GPIO_Port;
  relay.DISCHARGE_Pin = RELAY_DISCHARGE_Pin;
  relay.GND_SIG_GPIO_Port = RELAY_LS_GPIO_Port;
  relay.GND_SIG_Pin = RELAY_LS_Pin;
  relay.ON_SIG_GPIO_Port = RELAY_HS_GPIO_Port;
  relay.ON_SIG_Pin = RELAY_HS_Pin;
  relay.PRE_SIG_GPIO_Port = RELAY_PRECHARGE_GPIO_Port;
  relay.PRE_SIG_Pin = RELAY_PRECHARGE_Pin;

  //--- LIGHTS ---//
  lightsPeriph.CSPin0 = TMC5160_CS0_Pin;
  lightsPeriph.CSPort0 = TMC5160_CS0_GPIO_Port;
  lightsPeriph.CSPin1 = TMC5160_CS1_Pin;
  lightsPeriph.CSPort1 = TMC5160_CS1_GPIO_Port;

  lightsPeriph.PWR_EN_Pin = Light_ctrl_PWR_EN_Pin;
  lightsPeriph.PWR_EN_Port = GPIOK;

  lightsPeriph.TMC5160_SPI = &hspi5;

  lightsPeriph.right_ind_TIM = &htim2;
  lightsPeriph.right_ind_CH = TIM_CHANNEL_2;
  lightsPeriph.left_ind_TIM = &htim2;
  lightsPeriph.left_ind_CH = TIM_CHANNEL_1;
  lightsPeriph.master_TIM = &htim1;
  lightsPeriph.master_CH = TIM_CHANNEL_1;
  lightsPeriph.BRK_TIM = &htim5;
  lightsPeriph.BRK_CH = TIM_CHANNEL_2;
  lightsPeriph.DRL_TIM = &htim5;
  lightsPeriph.DRL_CH = TIM_CHANNEL_1;
  lightsPeriph.FLT_TIM = &htim3;
  lightsPeriph.FLT_CH = TIM_CHANNEL_1;

  //Initial state of lights: all off
  turn_off_indicators(&lightsPeriph, 1);
  turn_off_indicators(&lightsPeriph, 0);
  turn_off_DRL(&lightsPeriph);
  turn_off_brake_lights(&lightsPeriph);
  turn_off_fault_indicator(&lightsPeriph);
  turn_off_hazard_lights(&lightsPeriph);

  //--- COMMS ---//
  buart_main = B_uartStart(&huart4);
  btcp_main = B_tcpStart(BBMB_ID, &buart_main, buart_main, 1, &hcrc);
  buart_bms = B_uartStart(&huart8);
  btcp_bms = B_tcpStart(BBMB_ID, &buart_bms, buart_bms, 1, &hcrc);

  //--- FREERTOS ---//
  lightsCtrl = xQueueCreate(16, sizeof(uint8_t)); //Holds instruction for lights control
  relayCtrl = xQueueCreate(4, sizeof(uint8_t)); //Holds instruction to open (1) or close relay (2)

  configASSERT(xTaskCreate(lightsTask, "LightsTask", 1024, ( void * ) 1, 4, NULL));
  configASSERT(xTaskCreate(relayTask, "relayCtrl", 1024, ( void * ) 1, 4, NULL));

  configASSERT(xTimerStart(xTimerCreate("HeartbeatHandler",  pdMS_TO_TICKS(HEARTBEAT_INTERVAL / 2), pdTRUE, (void *)0, HeartbeatHandler), 0)); //Heartbeat handler
  configASSERT(xTimerStart(xTimerCreate("PSMTaskHandler",  pdMS_TO_TICKS(PSM_INTERVAL), pdTRUE, (void *)0, PSMTaskHandler), 0)); //Temperature and voltage measurements
  configASSERT(xTimerStart(xTimerCreate("BMSPeriodicReadHandler",  pdMS_TO_TICKS(BMS_READ_INTERVAL), pdTRUE, (void *)0, BMSPeriodicReadHandler), 0)); //Read from BMS periodically

  //--- RELAYS ---//
  close_relays(&relay);

  //--- ENABLE 12V FROM VICOR ---//
  HAL_GPIO_WritePin(BMS_NO_FLT_GPIO_Port, BMS_NO_FLT_Pin, 1);

  //--- BATTERY ---//
  batteryState = HEALTHY;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
#ifdef DEFAULT_TASK

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
#endif
  /* add threads, ... */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
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
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 13552;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1600;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  htim7.Init.Prescaler = 40000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
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
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOE, RELAY_HS_Pin|BMS_NO_FLT_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, RELAY_LS_Pin|RELAY_DISCHARGE_Pin|PSM_CS_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMS_WKUP_GPIO_Port, BMS_WKUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PSM_LVDS_EN_GPIO_Port, PSM_LVDS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, TMC5160_CS0_Pin|PSM_DReady_Pin|HORN_EN_Pin|TMC5160_CS1_Pin
                          |Light_ctrl_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, PSM_CS_1_Pin|PSM_CS_3_Pin|PSM_CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE5 PE6
                           PE7 PE8 PE9 PE10
                           PE11 PE13 PE15 PE0
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_HS_Pin BMS_NO_FLT_Pin LED2_Pin */
  GPIO_InitStruct.Pin = RELAY_HS_Pin|BMS_NO_FLT_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PI8 PI10 PI11 PI14
                           PI15 PI4 PI5 PI6
                           PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
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

  /*Configure GPIO pins : RELAY_LS_Pin RELAY_DISCHARGE_Pin PSM_CS_0_Pin */
  GPIO_InitStruct.Pin = RELAY_LS_Pin|RELAY_DISCHARGE_Pin|PSM_CS_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF3 PF4
                           PF5 PF6 PF7 PF8
                           PF9 PF10 PF11 PF12
                           PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_PRECHARGE_Pin */
  GPIO_InitStruct.Pin = RELAY_PRECHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_PRECHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESD_DETECT_Pin */
  GPIO_InitStruct.Pin = ESD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH2 PH3 PH4 PH5
                           PH7 PH8 PH12 PH13
                           PH14 PH15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA8 PA9
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15;
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
                           PJ12 PJ13 PJ14 PJ15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_FLT_Pin TMC5160_DIAG1_Pin TMC5160_DIAG0_Pin */
  GPIO_InitStruct.Pin = BMS_FLT_Pin|TMC5160_DIAG1_Pin|TMC5160_DIAG0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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

  /*Configure GPIO pin : BMS_WKUP_Pin */
  GPIO_InitStruct.Pin = BMS_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMS_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PSM_LVDS_EN_Pin */
  GPIO_InitStruct.Pin = PSM_LVDS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PSM_LVDS_EN_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : TMC5160_CS0_Pin PSM_DReady_Pin HORN_EN_Pin TMC5160_CS1_Pin
                           Light_ctrl_PWR_EN_Pin */
  GPIO_InitStruct.Pin = TMC5160_CS0_Pin|PSM_DReady_Pin|HORN_EN_Pin|TMC5160_CS1_Pin
                          |Light_ctrl_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pins : PSM_CS_1_Pin PSM_CS_3_Pin PSM_CS_2_Pin */
  GPIO_InitStruct.Pin = PSM_CS_1_Pin|PSM_CS_3_Pin|PSM_CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PK6 PK7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void serialParse(B_tcpPacket_t *pkt){
	switch(pkt->senderID){
		case DCMB_ID: //Parse data from DCMB
			if (pkt->data[0] == DCMB_LIGHTCONTROL_ID){
				xQueueSend(lightsCtrl, &(pkt->payload[1]), 0); //Send to lights control task

			} else if (pkt->data[0] == DCMB_RELAYS_STATE_ID){
				taskENTER_CRITICAL();

				if (pkt->payload[2] == OPEN){ //Open relays and resend
					open_relays(&relay);

					uint8_t buf[4] = {BBMB_RELAYS_STATE_ID, batteryState, OPEN, pkt->payload[3]};
					B_tcpSend(btcp_main, buf, sizeof(buf));

				} else if ((pkt->payload[2] == CLOSED) && (batteryState == HEALTHY)){ //Try to close relays
					close_relays(&relay);

					uint8_t buf[4] = {BBMB_RELAYS_STATE_ID, batteryState, CLOSED, pkt->payload[3]};
					B_tcpSend(btcp_main, buf, sizeof(buf));
				}

				taskEXIT_CRITICAL();

			} else if (pkt->data[0] == DCMB_STEERING_WHEEL_ID){
				//Horn
				if (pkt->data[2] & 0b00001000){ //Turn on horn
					HAL_GPIO_WritePin(HORN_EN_GPIO_Port, HORN_EN_Pin, GPIO_PIN_SET);
				} else if (~(pkt->data[2] & 0b00001000)){ //Turn off horn
					HAL_GPIO_WritePin(HORN_EN_GPIO_Port, HORN_EN_Pin, GPIO_PIN_RESET);
				}
			}
			break;

		case BMS_ID: //Parse data from BMS (comes from btcp_bms)
			if (pkt->data[0] == BMS_ERROR_STATUS){ //Received error from BMS
				uint8_t BMS_error[2 * 4] = {0};
				BMS_error[0] = BBMB_RELAYS_STATE_ID;
				BMS_error[1] = OPEN;
				if (pkt->data[1] == BMS_OV){	BMS_error[2] = 0x00;	}
				else if (pkt->data[1] == BMS_UV){	BMS_error[2] = 0x01;	}
				else if (pkt->data[1] == BMS_OT){	BMS_error[2] = 0x03;	}

				B_tcpSend(btcp_main, BMS_error, sizeof(BMS_error));

			} else { //Received SoCs, temps or voltage from BMS
				//Simply re-send on main bus
				B_tcpSend(btcp_main, pkt->data, sizeof(pkt->data));

				taskENTER_CRITICAL();
				if (BMS_requesting_from <= 6){
					//Update flags to indicate what data we've received
					if (pkt->data[0] == BMS_CELL_VOLT){
						BMS_data_received[0] = 1;
						float volt1 = arrayToFloat(pkt->data[4]);
						float volt2 = arrayToFloat(pkt->data[8]);
						float volt3 = arrayToFloat(pkt->data[12]);
						float volt4 = arrayToFloat(pkt->data[16]);
						float volt5 = arrayToFloat(pkt->data[20]);

//						char strToSend[100];
//						sprintf(strToSend, "Voltage1: %f\nVoltage2: %f\nVoltage3: %f\nVoltage4: %f\nVoltage5: %f\n", volt1, volt2, volt3, volt4, volt5);
//						HAL_UART_Transmit(&huart2, strToSend, sizeof(strToSend), 100);
					} else if (pkt->data[0] == BMS_CELL_TEMP){
						BMS_data_received[1] = 1;
					} else if (pkt->data[0] == BMS_CELL_SOC_ID){
						BMS_data_received[2] = 1;
					}

					//If all data from a BMS has been received, we are ready to read the next
					if ((BMS_data_received[0]) && (BMS_data_received[1]) && (BMS_data_received[2])){
						BMS_requesting_from += 1;
						BMS_data_received[0] = 0;
						BMS_data_received[1] = 0;
						BMS_data_received[2] = 0;

						uint8_t BMS_Request[2 * 4] = {0};
						BMS_Request[0] = BBMB_BMS_DATA_REQUEST_ID;
						BMS_Request[1] = BMS_requesting_from;
						floatToArray((float) battery_current, BMS_Request + 4);

 						B_tcpSendToBMS(btcp_bms, BMS_Request, sizeof(BMS_Request));
					}
				}
				taskEXIT_CRITICAL();
			}
			break;
		case CHASE_ID:
			if (pkt->data[0] == CHASE_LIGHTCONTROL_ID){
				xQueueSend(lightsCtrl, &(pkt->data[1]), 200); //Send to lights control task
			}
			break;

		case MCMB_ID:
			/*BBMB no need for MCMB data*/
			break;
	}
}



void relayTask(void * argument){
	uint8_t buf_relay[10];

	//When relays need to be opened, put a 1 in the queue. When they need to be closed, put a 2.

	for(;;){
		if (xQueueReceive(relayCtrl, &buf_relay, 200)){
			if (buf_relay[0] == 1){
//				open_relays(&relay);
			} else if (buf_relay[0] == 2){
				close_relays(&relay);
			}
		}
	}
}

void lightsTask(void * argument)
{
	uint8_t buf_get[10];
	double duty = 0.1;
	double blink = 50.82;
	double on_period = 160;

/**
 * Item in lightsCtrl queue is 8-bit binary instruction
 * 7: empty
 * 6: Start / stop (1/0)
 * 5: Fault indicator
 * 4: Hazard lights (left and right indicator)
 * 3: Brake Lights
 * 2: DRL
 * 1: Indicator
 * 0: Specifies left / right (0/1) indicator
 *
 * 0100 0010
 */

  /* Infinite loop */
  for(;;)
  {

    // osMessageQueueGet(lightsCtrlHandle, &buf_get, NULL, 200);
	if (xQueueReceive(lightsCtrl, &buf_get, 200)){
		// HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_6);
		uint8_t light_msg = buf_get[0];
		uint8_t light_id = light_msg & 0x3E;
		switch(light_id){ // mask 0b0011 1110 to isolate light
			case 0x02: // indicator
				if ((light_msg & 0x40) != 0x00){ // masking for start / stop bit
					turn_on_indicators(&lightsPeriph, (int)(light_msg & 0x01), duty, blink, on_period); // masking last bit for left or right
				}
				else{
					turn_off_indicators(&lightsPeriph, (int)(light_msg & 0x01)); // masking last bit for left or right
				}
				break;

			case 0x04: // DRL
				if ((light_msg & 0x40) != 0x00){
					turn_on_DRL(&lightsPeriph, duty);
				}
				else{
					turn_off_DRL(&lightsPeriph);
				}
				break;

			case 0x08: // brake
				if ((light_msg & 0x40) != 0x00){
					turn_on_brake_lights(&lightsPeriph, duty);
				}
				else{
					turn_off_brake_lights(&lightsPeriph);
				}
				break;

			case 0x10: // hazard light
				if ((light_msg & 0x40) != 0x00){
					turn_on_hazard_lights(&lightsPeriph, duty, blink);
				}
				else{
					turn_off_hazard_lights(&lightsPeriph);
				}
				break;

			case 0x20: // fault indicator - started by interrupt?
				if ((light_msg & 0x40) != 0x00){
					turn_on_fault_indicator(&lightsPeriph);
				}
				else{
					turn_off_fault_indicator(&lightsPeriph);
				}
				break;

			default:
				break;
		}
	}
  }
}

void HeartbeatHandler(TimerHandle_t xTimer){
	//Send periodic heartbeat so we know the board is still running
	B_tcpSend(btcp_main, heartbeat, sizeof(heartbeat));
	heartbeat[1] = ~heartbeat[1]; //Toggle for next time
	//Heartbeat only accessed here so no need for mutex
}

void PSMTaskHandler(TimerHandle_t xTimer){
//Battery


	double voltageCurrent_HV[2] = {0};
	uint8_t busMetrics_HV[3 * 4] = {0};
	busMetrics_HV[0] = BBMB_BUS_METRICS_ID;

	//PSMRead will fill first element with voltage, second with current
	PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 2, voltageCurrent_HV, 2); //Array output on channel #2

	//Update battery pack current global variable (used for BMS communication)
	taskENTER_CRITICAL();
	battery_current = voltageCurrent_HV[1];
	taskEXIT_CRITICAL();

	//Check overcurrent protection
	if (voltageCurrent_HV[1] >= HV_BATT_OVERCURRENT_DISCHARGE){ //Overcurrent protection --> Put car into safe state
		uint8_t car_state_error[1 * 4];
		car_state_error[0] = BBMB_RELAYS_STATE_ID;
		car_state_error[1] = OPEN;
		car_state_error[2] = 0x02; //Overcurrent

		B_tcpSend(btcp_main, car_state_error, sizeof(car_state_error));

//		if (PROTECTION_ENABLE){	open_relays(&relay);	}
	}

	floatToArray((float) voltageCurrent_HV[0], busMetrics_HV + 4); // fills 4 - 7 of busMetrics
	floatToArray((float) voltageCurrent_HV[1], busMetrics_HV + 8); // fills 8 - 11 of busMetrics

	B_tcpSend(btcp_main, busMetrics_HV, sizeof(busMetrics_HV));

//LV
	double voltageCurrent_LV[2] = {0};
	uint8_t busMetrics_LV[3 * 4] = {0};
	busMetrics_LV[0] = BBMB_LP_BUS_METRICS_ID;

	PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 1, voltageCurrent_LV, 2);
	floatToArray((float) voltageCurrent_LV[0], busMetrics_LV + 4); // fills 4 - 7 of busMetrics
	floatToArray((float) voltageCurrent_LV[1], busMetrics_LV + 8); // fills 16 - 19 of busMetrics

	B_tcpSend(btcp_main, busMetrics_LV, sizeof(busMetrics_LV));
}

void BMSPeriodicReadHandler(TimerHandle_t xTimer){
	/*Used to kickoff a new cycle of BMS data requests.
	* Request data from first BMS, and upon reception, serialParse will request from others
	*/
//	char junk[2] = {3, 1};
//	B_tcpSendToBMS(btcp_bms, junk, sizeof(junk));
	//B_tcpSend(btcp_main, junk, sizeof(junk));
	taskENTER_CRITICAL();
	if (BMS_requesting_from > 6){ //We've received from all BMS, start requesting from the first one again

		BMS_requesting_from = 1;

		uint8_t BMS_Request[2 * 4] = {0};
		BMS_Request[0] = BBMB_BMS_DATA_REQUEST_ID;
		BMS_Request[1] = BMS_requesting_from;
		floatToArray((float) battery_current, BMS_Request + 4); //Send current
		B_tcpSendToBMS(btcp_bms, BMS_Request, sizeof(BMS_Request));

	}
	taskEXIT_CRITICAL();
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
  for(;;)
  {
    osDelay(1);
  }
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

