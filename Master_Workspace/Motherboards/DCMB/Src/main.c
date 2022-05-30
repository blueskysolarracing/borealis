/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buart.h"
#include "btcp.h"
#include "h7Boot.h"
//#include "disp_renderer.h"
#include "ACE128Map.h"
#include "glcd.h"
#include "bglcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SPB
// GPIO PORT B: CAMERA, AUX2, FWD_REV, AUX1
// GPIO PORT C: AUX0, IGNITION, FAN
// GPIO PORT D: ARRAY
// (ARRAY, AUX0, IGNITION, FAN, CAMERA, AUX2, FWD_REV, AUX1)
#define AUX1 1
#define FWD_REV 2
#define AUX2 4
#define CAMERA 8
#define FAN 16
#define IGNITION 32
#define AUX0 64
#define ARRAY 128
#define BSSR_SPB_RX_BUFFER_SIZE 256


// SWB
// GPIO PORT B: SELECT, RIGHT, DOWN, LEFT, UP, <blank>, CRUISE_SIGNAL
// GPIO PORT C1: HORN_SIGNAL, RAD_SIGNAL, L_SIGNAL, R_SIGNAL
// GPIO PORT C2: ACC8 -> ACC1
#define SELECT 64
#define RIGHT 32
#define DOWN 16
#define LEFT 8
#define UP 4
#define CRUISE 1
#define HORN 8
#define RAD_SIG 4
#define L_SIG 2
#define R_SIG 1

//#define RIGHT_SIG 16
//#define HORN 128
//#define DPAD_DOWN 1
//#define DPAD_LEFT 2
//#define DPAD_RIGHT 8
//#define DPAD_UP 4
uint8_t LEFT_ENABLED = 0;
uint8_t RIGHT_ENABLED = 0;
uint8_t CENTER_ENABLED = 0;
uint8_t FRONT_ENABLED = 0;
uint8_t CAMERA_ENABLED = 0;
uint8_t CRUISE_MULT = 1;

uint8_t sidePanelData;
uint8_t steeringData[3];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

HRTIM_HandleTypeDef hhrtim;

LPTIM_HandleTypeDef hlptim1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;
SPI_HandleTypeDef hspi6;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

SRAM_HandleTypeDef hsram4;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
B_uartHandle_t* buart;
B_uartHandle_t* spbBuart;
B_uartHandle_t* swBuart;
B_tcpHandle_t* btcp;
uint8_t ignition_state = 0;
uint8_t array_state = 0;
SemaphoreHandle_t accMutex;
SemaphoreHandle_t motorButtonMutex;
uint8_t accValue = 0;
uint8_t brakeStatus = 0;
uint8_t motorState = 0;
uint8_t fwdRevState = 0;
uint8_t vfmUpState = 0;
uint8_t vfmDownState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI5_Init(void);
static void MX_SPI6_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART8_Init(void);
static void MX_FMC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_HRTIM_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void sidePanelTask(const void *pv);
static void steeringWheelTask(const void *pv);
static void displayTask();
static void lightsTmr(TimerHandle_t xTimer);
static void mc2StateTmr(TimerHandle_t xTimer);
//static void displayTmr(TimerHandle_t xTimer);
static void motCallback(uint8_t motState);
static void vfmUpCallback();
static void vfmDownCallback();
static void accResetCallback();
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
  arm_boot();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_UART8_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  MX_SPI2_Init(); //I don't know why it's not being called above
//  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); //Calibrate ADC (used for acceleration pedal)

  //--- DRIVER DISPLAYS ---//
  glcd_init();
  glcd_test_circles();

  HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_RESET);
	int defaultTest[4] = {420, 874, -454, 69};
	int defaultDetailed[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
//	drawP1Default(defaultTest);
//	drawP1Detailed(defaultDetailed);
//	drawP1Activate();
//	drawP1Deactivate();
	drawP1IgnitionOff();
//	drawP1BMSFault();

	int defaultTest2[4] = {1, 0, 1, 87};
	int defaultDetailed2[4] = {1, 2, 3, 4};
	int defaultBMSFault[4] = {89, 13, 13, 49};
	drawP2Default(defaultTest2);
	drawP2Detailed(defaultDetailed2);
	drawP2Activate();
	drawP2Deactivate();
	drawP2IgnitionOff(defaultBMSFault);
	drawP2BMSFault(defaultBMSFault);
  HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_SET);
//  displayInit(); //Legacy from GEN10
//  glcd_clear();

//  xTimerStart(xTimerCreate("lightsTimer", 666, pdTRUE, NULL, lightsTmr), 0);
  xTimerStart(xTimerCreate("mc2StateTimer", 20, pdTRUE, NULL, mc2StateTmr), 0);
//  xTimerStart(xTimerCreate("display", 200, pdTRUE, 0, displayTmr), 0);
  buart = B_uartStart(&huart4);
  spbBuart = B_uartStart(&huart3);
  swBuart = B_uartStart(&huart8);
  btcp = B_tcpStart(DCMB_ID, &buart, buart, 1, &hcrc);
  xTaskCreate(displayTask, "displayTask", 1024, 1, 5, NULL);
  xTaskCreate(sidePanelTask, "SidePanelTask", 1024, spbBuart, 5, NULL);
  xTaskCreate(steeringWheelTask, "SteeringWheelTask", 1024, swBuart, 5, NULL);
  disp_attachMotOnCallback(motCallback);
  disp_attachVfmUpCallback(vfmUpCallback);
  disp_attachVfmDownCallback(vfmDownCallback);
  disp_attachAccResetCallback(accResetCallback);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
#endif DEFAULT_TASK
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_1);
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
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief HRTIM Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM_Init(void)
{

  /* USER CODE BEGIN HRTIM_Init 0 */

  /* USER CODE END HRTIM_Init 0 */

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM_Init 1 */

  /* USER CODE END HRTIM_Init 1 */
  hhrtim.Instance = HRTIM1;
  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0xFFFD;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM_Init 2 */

  /* USER CODE END HRTIM_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim);

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_HIGH;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  hspi5.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  huart8.Init.BaudRate = 2000000;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM4 memory initialization sequence
  */
  hsram4.Instance = FMC_NORSRAM_DEVICE;
  hsram4.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram4.Init */
  hsram4.Init.NSBank = FMC_NORSRAM_BANK3;
  hsram4.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram4.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram4.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram4.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram4.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram4.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram4.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram4.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram4.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram4.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram4.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram4.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram4.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram4.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram4, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DISP_CS_1_Pin|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|Cam_Ctrl_Pin
                          |Screen_Ctrl_Pin|DISP_CS_0_Pin|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Fan_ctrl_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5|DISP_A0_Pin|DISP_LED_CTRL_Pin|DISP_RST_1_Pin
                          |DISP_RST_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISP_CS_1_Pin PE0 */
  GPIO_InitStruct.Pin = DISP_CS_1_Pin|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PI9 PI12 PI13 Cam_Ctrl_Pin
                           Screen_Ctrl_Pin DISP_CS_0_Pin PI4 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|Cam_Ctrl_Pin
                          |Screen_Ctrl_Pin|DISP_CS_0_Pin|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN0_Pin */
  GPIO_InitStruct.Pin = GPIO_IN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_IN0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PJ1 PJ2 PJ3 PJ4
                           PJ6 PJ7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : Fan_ctrl_Pin PG1 PG2 PG3
                           PG4 PG5 PG15 */
  GPIO_InitStruct.Pin = Fan_ctrl_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 PE12 PE13 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PJ5 DISP_A0_Pin DISP_LED_CTRL_Pin DISP_RST_1_Pin
                           DISP_RST_2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|DISP_A0_Pin|DISP_LED_CTRL_Pin|DISP_RST_1_Pin
                          |DISP_RST_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : PH12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN10_Pin */
  GPIO_InitStruct.Pin = GPIO_IN10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_IN10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PK2 PK3 PK4 PK5
                           PK6 PK7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void accResetCallback(){
	brakeStatus = 1;
}
static void vfmUpCallback(){
	static long lastVfmChange = 0;
	if(xTaskGetTickCount() > lastVfmChange + 500){
		lastVfmChange = xTaskGetTickCount();
		vfmUpState = 1;
	}
}
static void vfmDownCallback(){
	static long lastVfmChange = 0;
	if(xTaskGetTickCount() > lastVfmChange + 500){
		lastVfmChange = xTaskGetTickCount();
		vfmDownState = 1;
	}
}
static void motCallback(uint8_t motState){
	motorState =  motState;
}

static void lightsTmr(TimerHandle_t xTimer){
  static uint8_t current_state = 0;
  static uint8_t buf[4] = {0x03, 0x00, 0x00, 0x00};
  static uint8_t currentCameraState = 0;

//	current_state ^=1;
//	if(current_state&RIGHT_ENABLED){
//		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_13, GPIO_PIN_SET);
//		buf[2] = 0x01;
//		disp_setDCMBRightLightState(1);
//	} else {
//		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_13, GPIO_PIN_RESET);
//		buf[2] = 0x00;
//		disp_setDCMBRightLightState(0);
//	}
//	if(current_state&LEFT_ENABLED){
//		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);
//		buf[1] = 0x01;
//		disp_setDCMBLeftLightState(1);
//	} else {
//		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);
//		buf[1] = 0x00;
//		disp_setDCMBLeftLightState(0);
//	}
//	if(currentCameraState != CAMERA_ENABLED){
//		if(CAMERA_ENABLED){
//			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_14, GPIO_PIN_SET);
//		} else {
//			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_14, GPIO_PIN_RESET);
//		}
//		currentCameraState = CAMERA_ENABLED;
//	}
//	if((HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_9) == GPIO_PIN_SET)){
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
//		buf[3] = 0x01;
//		brakeStatus = 1;
//		disp_setDCMBStopLightState(1);
//	} else {
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
//		buf[3] = 0x00;
//		brakeStatus = 0;
//		disp_setDCMBStopLightState(0);
//	}
//	B_tcpSend(btcp, buf, 4);
}

static void mc2StateTmr(TimerHandle_t xTimer){
	static uint8_t started = 0;
	static char buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static int currentValue = 0;
	int positiveTurn = 0;
	int negativeTurn = 0;
	int difference = 0;
	static int accel_value = 0;
	static int regen_value = 0;
	uint16_t accel_reading_upper_bound = 40000; //ADC reading corresponding to 100% power request
	uint16_t accel_reading_lower_bound = 5000; //ADC reading corresponding to 0% power request
	uint16_t regen_reading_upper_bound = 40000; //ADC reading corresponding to 100% regen request
	uint16_t regen_reading_lower_bound = 5000; //ADC reading corresponding to 0% regen request
    uint32_t pedalsReading[2] = {0, 0};


    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    pedalsReading[0] = HAL_ADC_GetValue(&hadc1);
    pedalsReading[1] =  HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

	//TR: I'm not quite sure what this does...
//	int accValTemp = accValue == 255 ? currentValue : accValue;
//	if(!started){
//		started++;
//		currentValue = accValTemp;
//	} else{
//		positiveTurn = (currentValue + 63) % 128;
//		negativeTurn = (currentValue - 64) % 128;
//		if(positiveTurn > currentValue){
//			if(accValTemp <= positiveTurn && accValTemp >= currentValue){
//				difference = accValTemp - currentValue;
//			} else {
//				if(accValTemp < currentValue){
//					difference = accValTemp - currentValue;
//				} else {
//					difference = -(128 -(accValTemp - currentValue));
//				}
//			}
//		} else {
//			if(accValTemp <= currentValue && accValTemp >= negativeTurn){
//				difference = -(currentValue - accValTemp);
//			} else {
//				if(currentValue < accValTemp){
//					difference = accValTemp - currentValue;
//				} else {
//					difference = 128 - (currentValue - accValTemp);
//				}
//			}
//		}
//		difference = difference < 0 ? -difference * difference : difference * difference;
//		outputVal += difference;
////		sprintf(buf2, "c=%d,w=%d,d=%d,o=%d\r\n", currentValue, accValTemp, difference, outputVal);
//
//		currentValue = accValTemp;

// -------- FORWARD/REVERSE -------- //
		taskENTER_CRITICAL();
		fwdRevState= (sidePanelData & 0b00100000) >> 5; //Fill in 4th MSb of MC2_state "5 digital Buttons" byte (2nd of payload) with state of fwd/reverse switch on SPB
		taskEXIT_CRITICAL();

// -------- ACCELERATION -------- //
		//Get pedals reading, need to do it polling to ensure we have the latest measurements
		accel_value = (pedalsReading[0] - accel_reading_lower_bound) / (accel_reading_upper_bound - accel_reading_lower_bound) / 256; //Grab latest ADC reading of pedal position and map it to 0-255 scale by dividing by 2^8 (16 bit ADC)

		//Bound acceleration value
		if(accel_value < 0){
			accel_value = 0;
		} else if (accel_value > 0xff){
			accel_value = 0xff;
		}
		if(brakeStatus){
			accel_value = 0;
		}
		if(!motorState){
			accel_value = 0;
		}
		buf[2] = accel_value; //This is the value sent to the MCMB to control the power to the motor (0-255)

// -------- BUTTONS -------- //
		buf[1] = motorState << 4;
		buf[1] |= fwdRevState << 3;
		buf[1] |= vfmUpState << 2;
		buf[1] |= vfmDownState << 1;
		if(vfmUpState == 1){
			vfmUpState = 0;
		}
		if(vfmDownState == 1){
			vfmDownState = 0;
		}

// -------- REGEN -------- //
		regen_value = (pedalsReading[1] - regen_reading_lower_bound) / (regen_reading_upper_bound - regen_reading_lower_bound) / 256; //Grab latest ADC reading of pedal position and map it to 0-255 scale by dividing by 2^8 (16 bit ADC)

		//Bound regen value
		if(regen_value < 0){
			regen_value = 0;
		} else if (regen_value > 0xff){
			regen_value = 0xff;
		}
		if(brakeStatus){
			regen_value = 0;
		}
		if(!motorState){
			regen_value = 0;
		}

		buf[3] = regen_value;

		//Check if we need to turn on braking lights
		if (regen_value >= 10){ //If braking power is >= 10/255%, turn on break lights
	        uint8_t bufh[2] = {0x03, 0b00001000}; //[DATA ID, LIGHT INSTRUCTION]
	        B_tcpSend(btcp, bufh, 2);
		}

//		// TODO other buttons
//		disp_setDCMBAccPotPosition(accel_value);
//		B_tcpSend(btcp, buf, 8);
////		HAL_UART_Transmit_IT(&huart2, buf2, strlen(buf2));
//		// TODO other buttons
//		disp_setDCMBAccPotPosition(outputVal);
//		B_tcpSend(btcp, buf, 8);
}

void ignition_check(uint8_t data){
	static long ignition_press_time =  0;
	//static uint8_t ignition_state_inner = 0;
	static uint8_t button_pressed = 0;
	taskENTER_CRITICAL(); // data into global variable -> enter critical section
	if(!(data&IGNITION)){
		if(ignition_press_time == 0){
			ignition_press_time = xTaskGetTickCount();
		} else if ((ignition_press_time + 1000 < xTaskGetTickCount()) && button_pressed == 0){
			if(array_state == 1){ // PEKAC Mitigation
				array_state = 0;
				uint8_t array_buf[2] = {0x02, array_state};
				B_tcpSend(btcp, array_buf, 2);
				vTaskDelay(500);
			}
			ignition_press_time = 0;
			ignition_state ^= 1;
			uint8_t buf[2] = {0x01, ignition_state};
			B_tcpSend(btcp, buf, 2);
			button_pressed = 1;
		}
	} else {
		ignition_press_time = 0;
		button_pressed = 0;
	}
	taskEXIT_CRITICAL();
}

void array_check(uint8_t data){
	static long array_press_time = 0;
	static uint8_t button_pressed = 0;
	static uint8_t hornON = 0;
	uint8_t buf[10];
	static long fwdRevPressTime = 0;
	static uint8_t fwdRevPressed = 0;
	taskENTER_CRITICAL(); // data into global variable -> enter critical section
	if(!(data&ARRAY)){
		if(array_press_time == 0){
			array_press_time = xTaskGetTickCount();
		} else if ((array_press_time + 1000 < xTaskGetTickCount()) && button_pressed == 0){
			if(ignition_state != (uint8_t) 1){
				button_pressed = 0;
				array_press_time = 0;
				// Change led to show invalid
				return;
			}
			array_press_time = 0;
			array_state ^=1;
			uint8_t buf[2] = {0x02, array_state};
			B_tcpSend(btcp, buf, 2);
			button_pressed = 1;
		}
	} else {
		array_press_time = 0;
		button_pressed = 0;
	}
	if(!(data&FWD_REV)){
		if(fwdRevPressTime == 0){
			fwdRevPressTime = xTaskGetTickCount();
		} else if ((fwdRevPressTime + 1000 < xTaskGetTickCount()) && fwdRevPressed == 0){
			fwdRevPressTime = 0;
			fwdRevPressed = 0;
			fwdRevState ^= 1;
			disp_setMCMBFwdRev(!fwdRevState);
		}
	} else {
		fwdRevPressTime = 0;
		fwdRevPressed = 0;
	}
	taskEXIT_CRITICAL();
}

static void buttonCheck(uint8_t state){
// side panel
// (ARRAY, AUX0, IGNITION, FAN, CAMERA, AUX2, FWD_REV, AUX1)
  ignition_check(state);
  array_check(state);
  static uint8_t consistent_count;
  static uint8_t prev_data = 0;
  static uint8_t horn_on = 0;
  uint8_t data = state;
  taskENTER_CRITICAL(); // enter critical section
  if(data != 255){
	  if(!(data&FAN) && !horn_on){
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); // write 1 when switch is 0
//		  uint8_t buf[2] = {0x04, 0x01};
//		  B_tcpSend(btcp, buf, 2);
//		  horn_on = 1;
	  }
	  if(prev_data == data){
		  consistent_count++;
	  } else {
		  consistent_count = 0;
	  }
	  if(consistent_count ==  2){
		  if(!(data&AUX0)){
			  LEFT_ENABLED ^=1;
		  }
		  if(!(data&AUX2)){
			  RIGHT_ENABLED ^=1;
		  }
		  if(!(data&AUX1)){
			  if(RIGHT_ENABLED && LEFT_ENABLED){
				  RIGHT_ENABLED = 0;
				  LEFT_ENABLED = 0;
			  } else {
				  RIGHT_ENABLED = 1;
				  LEFT_ENABLED = 1;
			  }
		  }
		  if(!(data&CAMERA)){
			  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_14, GPIO_PIN_SET); // write 1 to camera when switch is 0
			  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_15, GPIO_PIN_SET); // write 1 to screen when switch is 0

//			  CAMERA_ENABLED ^=1;
		  }
	  }
  } else if (horn_on){
	  uint8_t bufh2[2] = {0x04, 0x00};
	  B_tcpSend(btcp, bufh2, 2);
	  horn_on = 0;
  }
  prev_data = data;
  taskEXIT_CRITICAL();
}

static void steeringButtonCheck(uint8_t *state){
	static long motor_press_time = 0;
	static uint8_t motor_pressed = 0;
	static uint8_t motor_state = 0;
	static uint8_t buf[2];
	static uint8_t horn_on = 0;
	static uint8_t vfm_up_pressed = 0;
	static long vfm_up_press_time = 0;
	static uint8_t vfm_down_pressed = 0;
	static long vfm_down_press_time = 0;
	static long last_vfm_change_time = 0;
//	if(!(state[0]&DPAD_DOWN)){
//		if(motor_press_time == 0){
//			motor_press_time = xTaskGetTickCount();
//		} else if((motor_press_time + 1000 < xTaskGetTickCount()) && motor_pressed == 0){
////			if(ignition_state != (uint8_t) 1){
////				motor_pressed = 0;
////				motor_press_time = 0;
////				return;
////			}
//			motor_pressed = 0;
//			motor_press_time = 0;
//			motor_state ^= 1;
//			motorState = motor_state;
//		}
//
//	} else {
//		motor_press_time = 0;
//		motor_pressed = 0;
//	}
	if(!(state[0]&HORN)){
	  if(!horn_on){
        uint8_t bufh[2] = {0x04, 0x01};
	    B_tcpSend(btcp, bufh, 2);
	    horn_on = 1;
	  }
	} else if (horn_on){
      uint8_t bufh2[2] = {0x04, 0x00};
	  B_tcpSend(btcp, bufh2, 2);
	  horn_on = 0;
	}

//	if(!(state[0]&DPAD_LEFT)){
//		if(vfm_up_press_time == 0){
//			vfm_up_press_time = xTaskGetTickCount();
//		} else if ((vfm_up_press_time + 1000 < xTaskGetTickCount()) && vfm_up_pressed == 0){
//			vfm_up_press_time = 0;
//			if(last_vfm_change_time + 500 < xTaskGetTickCount()){
//				vfmUpState ^= 1;
//				last_vfm_change_time = xTaskGetTickCount();
//			}
//		}
//	} else {
//		vfm_up_press_time = 0;
//		vfm_up_pressed = 0;
//	}
//
//	if(!(state[0]&DPAD_RIGHT)){
//		if(vfm_down_press_time == 0){
//			vfm_down_press_time = xTaskGetTickCount();
//		} else if ((vfm_down_press_time + 1000 < xTaskGetTickCount()) && vfm_down_pressed == 0){
//			vfm_down_press_time = 0;
//			if(last_vfm_change_time + 500 < xTaskGetTickCount()){
//				vfmDownState ^= 1;
//				last_vfm_change_time = xTaskGetTickCount();
//			}
//		}
//	} else {
//		vfm_down_press_time = 0;
//		vfm_down_pressed = 0;
//	}
	disp_updateNavState(!(state[0]&UP), !(state[0]&DOWN), !(state[0]&RIGHT), !(state[0]&LEFT), !(state[1]), state[2]);

}

static void steeringWheelTask(const void *pv){
// {0xa5, 0x03, DATA_1, DATA_2, DATA_3, CRC}
// DATA_1: [ACC8, ACC7, ACC6, ACC5, ACC4, ACC3, ACC2, ACC1] <-- ROTARY ENCODER DATA
// DATA_2: [x, x, x, CRUISE, HORN, RADIO, RIGHT_INDICATOR, LEFT_INDICATOR]
// DATA_3: [x, x, x, SELECT, RIGHT, LEFT, DOWN, UP]

  B_tcpHandle_t* btcp = pv;
  B_bufQEntry_t *e;
  uint8_t input_buffer[MAX_PACKET_SIZE + 4];
  uint8_t started = 0;
  uint8_t pos = 0;
  uint8_t crcAcc = 0;
  uint32_t crc;
  uint32_t crcExpected;
  uint8_t oldSteeringData[3] = {0, 0, 0};

  for(;;){
	e = B_uartRead(swBuart);

	taskENTER_CRITICAL();
	//Save old data
	for (int i = 0; i < 3; i++){ oldSteeringData[i] = steeringData[i]; }

	//Update global data
	if (e->buf[0] == BSSR_SERIAL_START && e->buf[1] == 0x03){
		for (int i = 0; i < 3; i++){
			if (steeringData[i] != e->buf[2 + i]){ steeringData[i] = e->buf[2 + i]; } //Only update if different (it should be different)
		}
	}
	}

	//---------- Process data ----------//
	// Navigation <- Not implemented
	// Cruise <- Not implemented

	//INDICATOR LIGHTS - SEND TO BBMB
    //Left indicator - SEND TO BBMB
    if ((steeringData[1] & 0b00000001) != (oldSteeringData[1] & 0b00000001)){ //If the state of the left indicator changed, send new data to BBMB
    	//If LEFT_INDICATOR == 1 --> Retract lights
    	//If LEFT_INDICATOR == 0 --> Extend lights
        uint8_t bufh[2] = {0x03, 0x00}; //[DATA ID, LIGHT INSTRUCTION]

    	if ((steeringData[1] & 0b00000001) == 0){ //Extend left indicator
        	bufh[1] = 0b01000010;
    	} else { //Retract left indicator
        	bufh[1] = 0b00000010;
    	}
        B_tcpSend(btcp, bufh, 2);
    }

    //Right indicator - SEND TO BBMB
    else if ((steeringData[1] & 0b00000010) != (oldSteeringData[1] & 0b00000010)){ //If the state of the right indicator changed, send new data to BBMB
    	//If RIGHT_INDICATOR == 1 --> Retract lights
    	//If RIGHT_INDICATOR == 0 --> Extend lights
        uint8_t bufh[2] = {0x03, 0x00}; //[DATA ID, LIGHT INSTRUCTION]

    	if ((steeringData[1] & 0b00000010) == 0){ //Extend right indicator
        	bufh[1] = 0b01000011;
    	} else { //Retract right indicator
        	bufh[1] = 0b00000011;
    	}
        B_tcpSend(btcp, bufh, 2);
    }

	//Horn - SEND TO BBMB
    else if ((steeringData[1] & 0b00001000) != (oldSteeringData[1] & 0b00001000)){
        uint8_t bufh[2] = {0x04, 0x00}; //[DATA ID, HORN STATE]

    	if ((steeringData[1] & 0b00001000) == 0){ //Turn on horn
            bufh[1] = 0x01; //[DATA ID, HORN STATE]
    	} else { //Turn off horn
            bufh[1] = 0x00; //[DATA ID, HORN STATE]
    	}
        B_tcpSend(btcp, bufh, 2);
    }

    //Encoder - SEND TO MCMB
    else if ((steeringData[0] & 0b11111111) != (oldSteeringData[0] & 0b11111111)){
    	uint8_t old_ang = encoderMap8[oldSteeringData[0]];
    	uint8_t new_ang = encoderMap8[steeringData[0]];

    	uint8_t target = CRUISE_MULT * (new_ang - old_ang);

        uint8_t bufh[2] = {0x04, target}; //[DATA ID, angle]
        B_tcpSend(btcp, bufh, 2);
    }

    //Cruise - (Try to change) Motor state and send to MCMB
    else if ((steeringData[1] & 0b00010000) != (oldSteeringData[1] & 0b00010000)){

    	//To be implemented




    //Radio - Enable driver voice radio
    } else if ((steeringData[1] & 0b00000100) != (oldSteeringData[1] & 0b00000100)){
    	//No plan to implement it in GEN11
    }

    //Select button pressed - Cycle through frames on driver display
    else if ((steeringData[2] & 0b00010000) != (oldSteeringData[2] & 0b00010000)){

    }

    //Up button pressed
    else if ((steeringData[2] & 0b00000001) != (oldSteeringData[2] & 0b00010000)){
    	//No plan to implement it in GEN11
    }

    //Down button pressed
    else if ((steeringData[2] & 0b00000010) != (oldSteeringData[2] & 0b00010000)){
    	//No plan to implement it in GEN11

    }

    //Left button pressed
    else if ((steeringData[2] & 0b00000100) != (oldSteeringData[2] & 0b00010000)){
    	//No plan to implement it in GEN11

    }

    //Right button pressed
    else if ((steeringData[2] & 0b00001000) != (oldSteeringData[2] & 0b00010000)){
    	//No plan to implement it in GEN11

    }

	B_uartDoneRead(e);

	taskEXIT_CRITICAL(); // exit critical section

	// print statement -> connect to serial monitor
    char buffer[100];
    sprintf(buffer, "STEERING WHEEL TEST\r\n");
    HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 100);
    for(int i = 0; i < 2; i++){
    	for(int j = 1 << 7; j > 0; j /= 2){
    		if((steeringData[i] & j) != 0){
    			sprintf(buffer, "1");
    			HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 100);
    			HAL_Delay(100);
    		}
    		else{
    			sprintf(buffer, "0");
    			HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 100);
    			HAL_Delay(100);
    		}
    	}
    }
}


static void sidePanelTask(const void *pv){
// {0xa5, 0x04, sidePanelData, CRC};
// sidePanelData is formatted as [IGNITION, CAMERA, FWD/REV, FAN, AUX2, AUX1, AUX0, ARRAY]

  B_bufQEntry_t *e;
  uint8_t input_buffer[MAX_PACKET_SIZE + 4];
  uint8_t started = 0;
  uint8_t pos = 0;
  uint8_t crcAcc = 0;
  uint32_t crc;
  uint32_t crcExpected;

  for(;;){
    e = B_uartRead(spbBuart);

	taskENTER_CRITICAL(); // data into global variable -> enter critical section

	if (e->buf[0] == BSSR_SERIAL_START && e->buf[1] == 0x04){
		if (sidePanelData != e->buf[2]){ sidePanelData = e->buf[2]; } //Only update if different (it should be different)
	} else {
		return; //Data received was not from side panel
	}

	//Check CRC, optional

    B_uartDoneRead(e);

  //---------- Process data ----------//
  //REAR CAMERA AND SCREEN
  if (sidePanelData & (1 << 6)){
	  HAL_GPIO_WritePin(GPIOI, Cam_Ctrl_Pin, GPIO_PIN_SET); //Enable camera
	  HAL_GPIO_WritePin(GPIOI, Screen_Ctrl_Pin, GPIO_PIN_SET); //Enable screen
  } else {
	  HAL_GPIO_WritePin(GPIOI, Cam_Ctrl_Pin, GPIO_PIN_RESET); //Disable camera
	  HAL_GPIO_WritePin(GPIOI, Screen_Ctrl_Pin, GPIO_PIN_RESET); //Disable screen

  }

  //FAN
  if (sidePanelData & (1 << 4)){
	  HAL_GPIO_WritePin(GPIOG, Fan_ctrl_Pin, GPIO_PIN_SET); //Enable fan
  } else {
	  HAL_GPIO_WritePin(GPIOG, Fan_ctrl_Pin, GPIO_PIN_RESET); //Disable fan
  }

  //AUX0 (DRL in GEN11)
  uint8_t bufh[2] = {0x03, 0x00}; //[DATA ID, LIGHT INSTRUCTION]

  if (sidePanelData & (1 << 1)){
	bufh[1] = 0b00000100; //AUX0 == 1 -> DRL off
  } else { //Turn off horn
	bufh[1] = 0b01000100; //AUX0 == 0 -> DRL on
  }

  //AUX1 (not implemented in GEN11)
  if (sidePanelData & (1 << 2)){
	  HAL_Delay(100);
  } else { //Turn off horn
	  HAL_Delay(100);
  }

  //AUX2 (not implemented in GEN11)
  if (sidePanelData & (1 << 3)){
	  HAL_Delay(100);
  } else { //Turn off horn
	  HAL_Delay(100);
  }

  //ARRAY (close solar array power relays (if allowed))
  //To be implemented
  if (sidePanelData & (1 << 0)){
	  HAL_Delay(100);
  } else { //Turn off horn
	  HAL_Delay(100);
  }

  //FWD/REV (change motor direction forward or reverse and turn on displays if reverse)
  //To be implemented
  if (sidePanelData & (1 << 5)){
	  HAL_GPIO_WritePin(GPIOI, Cam_Ctrl_Pin, GPIO_PIN_SET); //Enable camera
	  HAL_GPIO_WritePin(GPIOI, Screen_Ctrl_Pin, GPIO_PIN_SET); //Enable screen

	  //Send new motor state (if allowed)
  } else {
	  HAL_GPIO_WritePin(GPIOI, Cam_Ctrl_Pin, GPIO_PIN_RESET); //Disable camera
	  HAL_GPIO_WritePin(GPIOI, Screen_Ctrl_Pin, GPIO_PIN_RESET); //Disable screen

	  //Send new motor state (if allowed)
  }

  //IGNITION (put car into drive mode (if allowed)
  //To be implemented
  if (sidePanelData & (1 << 7)){
	  HAL_Delay(100);
  } else {
	  HAL_Delay(100);
  }

  B_tcpSend(btcp, bufh, 2);

  taskEXIT_CRITICAL(); // exit critical section
}
}

void serialParse(B_tcpPacket_t *pkt){
	switch(pkt->sender){
	case 0x01:
		  if(pkt->payload[4] == 0x02){
			  uint32_t value = pkt->payload[7];
			  value |= (pkt->payload[7] << 8);
			  if(pkt->payload[5] == 0x01){
				 // setBBMBBmsAlertType(DISP_BMS_ALERT_BUS_OV, value );
			  } else if (pkt->payload[5] == 0x02){
				//  setBBMBBmsAlertType(DISP_BMS_ALERT_BUS_UV, value );
			  } else if (pkt->payload[6] == 0x03){
				 // setBBMBBmsAlertType(DISP_BMS_ALERT_CELL_OT, value );
			  } else if (pkt->payload[7] == 0x04){
				 // setBBMBBmsAlertType(DISP_BMS_ALERT_CELL_UT, value );
			  }
		  }
		  break;
	case 0x03:
		if(pkt->payload[4] == 0x03){
			disp_setMCMBPulseFreq(pkt->payload[7]*2);
		}
	}
}

void displayTask(TimerHandle_t xTimer){
//	drawP1();
	drawP2();

	vTaskDelay(pdMS_TO_TICKS(100)); //Every 100ms

//	uint32_t id = pvTimerGetTimerID(xTimer);
//	if(id == 0){
//		vTimerSetTimerID(xTimer, 1);
//	}else if(id == 1){
//		//Draw to displays
//		drawP1();
//		drawP2();
//	}else if(id == 2){
//		xTimerChangePeriod(xTimer, 25, portMAX_DELAY);
//		vTimerSetTimerID(xTimer, 3);
//	}else{
////		if(xSemaphoreTake(dispMtx, 0) == pdPASS){
////			lv_task_handler();
////			xSemaphoreGive(dispMtx);
////		}
//	}
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

