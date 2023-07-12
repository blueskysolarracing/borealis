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
#include "h7Boot.h"
#include "ACE128Map.h"
#include "glcd.h"
#include "bglcd.h"
#include "math.h"
#include "battery_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//--- COMMS ---//
#define CONNECTION_EXPIRY_THRESHOLD 2000 //Number of ticks since last packet received before connection is considered "lost"

//--- PPTMB ---//
#define IGNORE_PPTMB 1 //Ignore PPTMB for display purpose when IGNORE_PPTMB == 1

//--- DISPLAY ---//
#define SLEEP_FRAME_EN 1 //When set to 1, enable the sleeping car start frame (when neither relays are closed)
#define DISP_REFRESH_DELAY 200 //Period between refreshes of driver display (in ms)
#define PEDALS_REFRESH_PERIOD 50 //Period between sending new pedal measurements (in ms)
#define HEARTBEAT_INTERVAL 1000 //Period between heartbeat (in ms)
#define LOW_SUPP_VOLT_DISP_ALERT_EN 0 //Enable an alert that displays a warning that the supplemental battery is low (needed for ASC specifically)
#define LOW_SUPP_VOLT_THRESHOLD 11.0 //Threshold at which to display low supplemental battery voltage

//--- PEDALS ---//
#define PEDAL_PULLUP 1.0 //Pullup resistance of pedal ADC input (kR)
#define ACCEL_ZERO_RESISTANCE 0.358 //Resistance of acceleration pedal when not pressed
#define REGEN_ZERO_RESISTANCE 2.3 //Resistance of regen pedal when not pressed
#define ACCEL_PEDAL_SLOPE 0.166 //Resistance per degree, empirically f`ound with delta-resistance / delta-angle
#define REGEN_PEDAL_SLOPE 0.25 //Resistance per degree, empirically found with delta-resistance / delta-angle
#define PEDALS_MEASUREMENT_INTERVAL 20 //Measure pedals every PEDALS_MEASUREMENT_INTERVAL ms
#define ADC_NUM_AVG 30.0
//#define USE_ADC_REGEN

//--- MOTOR ---//
enum CRUISE_MODE {
	CONSTANT_POWER,
	CONSTANT_SPEED
};

#define MOTOR_DATA_PERIOD 20 //Send motor data every MOTOR_DATA_PERIOD (ms)
#define MAX_VFM 8 //Maximum VFM setting
#define CRUISE_MODE CONSTANT_SPEED //Specifies how how cruise control should work (maintains constant motorTargetPower or maintains motorTargetSpeed)
/* ^ Need to update in MCMB as well ^ */
#define REGEN_DEFAULT_VALUE_STEERING_WHEEL 255 // regen value when pressing the regen button on the steering wheel
#define REGEN_BATTERY_VOLTAGE_THRESHOLD 120 // voltage above which regen should be disabled
#define REGEN_BATTERY_CELL_VOLTAGE_THRESHOLD 4

//--- SPB/SWB ---//
#define BSSR_SPB_SWB_ACK 0x77 //Acknowledge signal sent back from DCMB upon reception of data from SPB/SWB (77 is BSSR team number :D)

//--- BATTERY ---//

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

HRTIM_HandleTypeDef hhrtim;

LPTIM_HandleTypeDef hlptim1;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

WWDG_HandleTypeDef hwwdg1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
//--- PEDALS ---//
float accelReading = -1;
float accelValue = 0.0;

//--- DISPLAY ---//
uint8_t refresh_display = 1; //Flag to initiate refreshing of display
uint8_t pToggle = 0; //Needed to choose which display to write data to
uint8_t display_selection = 0; //Select which frame to display
uint8_t battery_faulted_display_selection = 7; //Select which frame to display when battery is faulted
struct disp_common common_data = {0}; //Three global structs where data is written to
struct disp_default_frame default_data = {0};
struct disp_detailed_frame detailed_data = {0};

//--- COMMS ---//
B_uartHandle_t* buart;
B_uartHandle_t* spbBuart;
B_uartHandle_t* swBuart;
B_tcpHandle_t* btcp;
uint8_t heartbeat[2] = {DCMB_HEARTBEAT_ID, 0};

uint8_t ignition_state = 0;
uint8_t array_state = 0;

//Initialize the tick count for each to
uint32_t BBMB_last_packet_tick_count 	= 0;
uint32_t PPTMB_last_packet_tick_count 	= 0;
uint32_t MCMB_last_packet_tick_count 	= 0;
uint32_t BMS_last_packet_tick_count 	= 0;
uint32_t Chase_last_packet_tick_count 	= 0;

uint8_t BBMBFirstPacketReceived = 0;


//--- MOTOR ---//
typedef enum {
	OFF,
	PEDAL,
	CRUISE,
	REGEN,
	STANDBY
} MOTORSTATE;
uint16_t motorTargetPower = 0; // value from 0 - 256
uint8_t brakeStatus = 0;
uint8_t motorState = 0;
uint8_t fwdRevState = 0;
uint8_t ecoPwrState = 0; //0 is ECO, 1 is POWER
uint8_t vfmUpState = 0;
uint8_t vfmDownState = 0;
uint8_t motorTargetSpeed = 0; // maintain current speed of car when cruise control is enabled by pressing on the steering wheel
uint8_t brakePressed = 0;

uint8_t start_steering_wheel_constant_regen = 0; // 1 is on, 0 is off

typedef enum {
	BRAKE_PRESSED,
	BRAKE_RELEASED,
} BRAKESTATE;

//--- SIDE PANEL BOARD ---//
typedef enum {
	IGNITION_ON,
	IGNITION_OFF,
} IGNITIONSTATE;

uint8_t ignitionState = IGNITION_OFF;

//--- STEERING WHEEL ---//
uint8_t steeringData[3];
uint8_t oldLeftButton;
uint8_t oldRightButton;
uint8_t oldUpButton;
uint8_t oldDownButton;
uint8_t oldMiddleButton;
uint8_t steering_wheel_variable_regen_value = 0;


//--- BATTERY ---//
float batteryVoltage = 0;
uint8_t batteryState = HEALTHY;
uint8_t previousBatteryState = HEALTHY;
uint8_t batteryRelayState = OPEN;
float battery_cell_voltages[NUM_BATT_CELLS]; //Array of the voltages of each 14P group
float battery_soc[NUM_BATT_CELLS]; //Array of the SoCs of each 14P group (5 cells per BMS)
float battery_temp[NUM_BATT_TEMP_SENSORS]; //Array of the temp of each thermistor (3 thermistors per BMS)
	/* Formats
	 * battery_cell_voltages = 	[VOLT0/BMS0, VOLT1/BMS0, VOLT2/BMS0, VOLT0/BMS1, VOLT1/BMS1, VOLT2/BMS1, ... VOLT0/BMS5, VOLT1/BMS5, VOLT2/BMS5]
	 * battery_soc = 			[SoC0/BMS0,  SoC1/BMS0,  SoC2/BMS0,  SoC0/BMS1,  SoC1/BMS1,  SoC2/BMS1,  ... SoC0/BMS5,  SoC1/BMS5,  SoC2/BMS5]
	 * battery_temp = 			[TEMP0/BMS0, TEMP1/BMS0, TEMP2/BMS0, TEMP0/BMS1, TEMP1/BMS1, TEMP2/BMS1, ... TEMP0/BMS5, TEMP1/BMS5, TEMP2/BMS5]
	 */
//--- ARRAY ---//
uint8_t arrayRelayState = OPEN;

//--- SIDE PANEL ---//
uint8_t camera_switch_is_on = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_RTC_Init(void);
static void MX_WWDG1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_HRTIM_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART8_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_QUADSPI_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void sidePanelTask(const void *pv);
static void displayTask(const void *pv);
static void steeringWheelTask(const void *pv);
static void pedalTask(const void* pv);
static void motorDataTimer(TimerHandle_t xTimer);
static void VFMSignalTimer(TimerHandle_t xTimer);

void HeartbeatHandler(TimerHandle_t xTimer);
float convertToAngle(uint16_t ADC_reading, uint8_t regen_or_accel); //Convert ADC code of regen and accelerator pedal to angle in degrees
void motorDataTask(TimerHandle_t xTimer);

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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_SPI3_Init();
  MX_UART8_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */
  //--- NUKE LED ---//
  HAL_TIM_Base_Start_IT(&htim7);

  //--- COMMS ---//
  buart = B_uartStart(&huart4);
  spbBuart = B_uartStart(&huart3);
  swBuart = B_uartStart(&huart8);
  btcp = B_tcpStart(DCMB_ID, &buart, buart, 1, &hcrc);

  //--- ADC ---//
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

  for (int i = 0; i < NUM_BATT_CELLS; ++i) {
	  detailed_data.min_cell_voltages[i] = 100000;
	  detailed_data.max_cell_voltages[i] = -100000;
  }

  for (int i = 0; i < NUM_BATT_TEMP_SENSORS; ++i) {
	  detailed_data.min_cell_temperatures[i] = 100000;
	  detailed_data.max_cell_temperatures[i] = -100000;
  }

  //--- FREERTOS ---//
  xTaskCreate(pedalTask, "pedalTask", 1024, ( void * ) 1, 4, NULL);
  xTaskCreate(displayTask, "displayTask", 1024, 1, 4, NULL);
  xTaskCreate(sidePanelTask, "SidePanelTask", 1024, spbBuart, 4, NULL);
  xTaskCreate(steeringWheelTask, "SteeringWheelTask", 1024, swBuart, 4, NULL);
  xTimerStart(xTimerCreate("motorDataTimer", pdMS_TO_TICKS(MOTOR_DATA_PERIOD), pdTRUE, NULL, motorDataTimer), 0); //Send data to MCMB periodically
  xTimerStart(xTimerCreate("HeartbeatHandler",  pdMS_TO_TICKS(HEARTBEAT_INTERVAL / 2), pdTRUE, (void *)0, HeartbeatHandler), 0); //Heartbeat handler

  //--- DRIVER DISPLAYS ---//
  glcd_init();

  //--- VFM ---//
  default_data.P2_VFM = 0;

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 5000);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
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
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
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
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 75-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim7.Init.Prescaler = 32499;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000;
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
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

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
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

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
  * @brief WWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG1_Init(void)
{

  /* USER CODE BEGIN WWDG1_Init 0 */

  /* USER CODE END WWDG1_Init 0 */

  /* USER CODE BEGIN WWDG1_Init 1 */

  /* USER CODE END WWDG1_Init 1 */
  hwwdg1.Instance = WWDG1;
  hwwdg1.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg1.Init.Window = 64;
  hwwdg1.Init.Counter = 64;
  hwwdg1.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG1_Init 2 */

  /* USER CODE END WWDG1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOE, DISP_CS_1_Pin|LED2_Pin|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|BACKUP_CAMERA_CTRL_Pin
                          |BACKUP_SCREEN_CTRL_Pin|DISP_CS_0_Pin|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, FAN_CTRL_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5|DISP_A0_Pin|DISP_LED_CTRL_Pin|DISP_RST_2_Pin
                          |DISP_RST_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISP_CS_1_Pin LED2_Pin PE0 */
  GPIO_InitStruct.Pin = DISP_CS_1_Pin|LED2_Pin|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PI9 PI12 PI13 BACKUP_CAMERA_CTRL_Pin
                           BACKUP_SCREEN_CTRL_Pin DISP_CS_0_Pin PI4 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|BACKUP_CAMERA_CTRL_Pin
                          |BACKUP_SCREEN_CTRL_Pin|DISP_CS_0_Pin|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : brakeDetect_Pin */
  GPIO_InitStruct.Pin = brakeDetect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(brakeDetect_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : FAN_CTRL_Pin PG1 PG2 PG3
                           PG4 PG5 PG15 */
  GPIO_InitStruct.Pin = FAN_CTRL_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 PE12 PE13 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PJ5 DISP_A0_Pin DISP_LED_CTRL_Pin DISP_RST_2_Pin
                           DISP_RST_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|DISP_A0_Pin|DISP_LED_CTRL_Pin|DISP_RST_2_Pin
                          |DISP_RST_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN10_Pin */
  GPIO_InitStruct.Pin = GPIO_IN10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_IN10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PK2 PK4 PK5 PK6
                           PK7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
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

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PI6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void pedalTask(const void* p) {
	float accel_r_0 = 0.3494; //Resistance when pedal is unpressed (kR)
	float accel_reading_upper_bound = 61000.0; //ADC reading corresponding to 0% power request
	float accel_reading_lower_bound = 24250.0; //ADC reading corresponding to 100% power request
	float accel_reading_threshold = 55.0; //Threshold at which the pedal won't respond (on 0-256 scale)
	uint8_t brakeState = BRAKE_RELEASED;
	uint8_t prevBrakeState = brakeState;
    uint8_t bufh2[2] = {DCMB_LIGHTCONTROL_ID, 0x00}; //[DATA ID, LIGHT INSTRUCTION]
	uint8_t firstTime = 1;
	uint8_t adc_regen_threshold = 30;

	uint8_t start_adc_regen = 0;
    while (1) {
#ifdef USE_ADC_REGEN
    	if (steering_wheel_variable_regen_value > adc_regen_threshold
    			&& -REGEN_BATTERY_CELL_VOLTAGE_THRESHOLD <= detailed_data.max_voltage / 10.0
				&& detailed_data.max_voltage / 10.0 <= REGEN_BATTERY_CELL_VOLTAGE_THRESHOLD
		) {
    		start_adc_regen = 1;
		} else {
			start_adc_regen = 0;
		}

#endif
		//--- PEDALS ADC READINGS ---//
		for (int i = 0; i < ADC_NUM_AVG; i++){
			vTaskSuspendAll();
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			float currentVal = HAL_ADC_GetValue(&hadc1);
			accelReading += currentVal;
			xTaskResumeAll();
		}

		// Check if brake is pressed or car is going to regen
		if (HAL_GPIO_ReadPin(brakeDetect_GPIO_Port, brakeDetect_Pin) == 0 || start_steering_wheel_constant_regen || start_adc_regen) {
			brakeState = BRAKE_PRESSED;
		} else {
			brakeState = BRAKE_RELEASED;
		}
		// send command to turn on/off brake lights if brakeState has changed
		// Need to wait until BBMB is known to be alive and then send first command
		if (BBMBFirstPacketReceived && (firstTime || brakeState != prevBrakeState)) {
			firstTime = 0;
			prevBrakeState = brakeState;
			if (brakeState == BRAKE_PRESSED) {
				// turn on brake lights
				bufh2[1] = 0b01001000;
			} else {
				// turn off brake lights
				bufh2[1] = 0b00001000;
			}
		    B_tcpSend(btcp, bufh2, sizeof(bufh2));
		}

		//Compute value on 0-256 scale
		accelValue = 256 - round(((accelReading/ADC_NUM_AVG) - accel_reading_lower_bound) / (accel_reading_upper_bound - accel_reading_lower_bound) * 256);

		//Bound acceleration value
		if (accelValue < 0){ //Deadzone of 15
			accelValue = 0;
		} else if (accelValue > 256 ){
			accelValue = 256;
		}

		//Try to catch if accel pedal cable is cut
		//Since pedal pot is pull-up, if the cable is cut, the ADC reading will be very low
		if ((accelReading/ADC_NUM_AVG) < 5000){
			accelValue = 0;
		}

		// setting global motorTargetPower, motorState
		vTaskSuspendAll();

		//Pedal has not effect when the motor is in cruise mode
		if (motorState != CRUISE){
#ifdef USE_ADC_REGEN
			if (start_adc_regen) {
				motorTargetPower = (uint16_t)steering_wheel_variable_regen_value;
				motorState = REGEN;
				default_data.P2_motor_state = REGEN;
			}
#else
			if (start_steering_wheel_constant_regen) {
				motorTargetPower = REGEN_DEFAULT_VALUE_STEERING_WHEEL;  // can change to any value we want
				motorState = REGEN;
				default_data.P2_motor_state = REGEN;
			}
#endif
			else if (accelValue >= accel_reading_threshold && brakeState == BRAKE_RELEASED) {
				motorTargetPower = (uint16_t) (accelValue - accel_reading_threshold) * (1.0 + accel_reading_threshold/255.0);
				motorState = PEDAL;
				default_data.P2_motor_state = PEDAL;
			} else { //Not in cruise and pedal isn't pressed, turn off motor
				motorTargetPower = (uint16_t) 0;
				motorState = STANDBY;
				default_data.P2_motor_state = STANDBY;
			}
		// Braking exits CRUISE mode if enabled.
		} else {
			if (brakeState == BRAKE_PRESSED) {
				motorTargetPower = (uint16_t) 0;
				motorState = STANDBY;
				default_data.P2_motor_state = STANDBY;
			}
		}

		//Turn off motor if needed. Overrides original motorState
		if (ignitionState != IGNITION_ON){
			motorState = OFF;
			motorTargetPower = (uint16_t) 0;
			default_data.P2_motor_state = OFF;
		}

		xTaskResumeAll();

		//Reset variables for next averaging
		accelReading = 0;

		osDelay(PEDALS_MEASUREMENT_INTERVAL);
	}
}

float convertToAngle(uint16_t ADC_reading, uint8_t regen_or_accel){
	//Convert ADC code of regen and accelerator pedal to angle in degrees

	if (regen_or_accel){ //Accel
		return (PEDAL_PULLUP * (ADC_reading / (65536.0 - 1.0)) - ACCEL_ZERO_RESISTANCE) / ACCEL_PEDAL_SLOPE;
	} else { //Regen
		return (PEDAL_PULLUP * (ADC_reading / (65536.0 - 1.0)) - REGEN_ZERO_RESISTANCE) / REGEN_PEDAL_SLOPE;
	}
}

void HeartbeatHandler(TimerHandle_t xTimer){
	//Send periodic heartbeat so we know the board is still running
	B_tcpSend(btcp, heartbeat, sizeof(heartbeat));
	heartbeat[1] = ~heartbeat[1]; //Toggle for next time
}

void serialParse(B_tcpPacket_t *pkt){
	vTaskSuspendAll();

	switch(pkt->sender){
	case PPTMB_ID:
		 //Update connection status
		 PPTMB_last_packet_tick_count = xTaskGetTickCount();

		 if (pkt->data[0] == PPTMB_BUS_METRICS_ID){ //HV bus
			common_data.solar_power = 			(short) round(arrayToFloat(&(pkt->data[4])) * arrayToFloat(&(pkt->data[8]))); //Solar power
			detailed_data.P1_solar_voltage = 	(short) round(arrayToFloat(&(pkt->data[4]))); //Solar voltage
			detailed_data.P1_solar_current = 	arrayToFloat(&(pkt->data[8])); //Solar current

			if (batteryState == FAULTED && fabs(detailed_data.P1_solar_current) > fabs(detailed_data.max_solar_current))
				detailed_data.max_solar_current = detailed_data.P1_solar_current;
		 } else if (pkt->data[0] == PPTMB_RELAYS_STATE_ID){ //Read relay state from PPTMB
			arrayRelayState = pkt->data[3]; //Display task will take care of choosing appropriate frame
			common_data.array_relay_state = arrayRelayState;
		 }
		 break;

	case MCMB_ID:
		 //Update connection status
		 MCMB_last_packet_tick_count = xTaskGetTickCount();

		if (pkt->data[0] == MCMB_BUS_METRICS_ID){ //HV bus
			common_data.motor_power = 			(short) round(arrayToFloat(&(pkt->data[4])) * arrayToFloat(&(pkt->data[8]))); //Motor power
			detailed_data.P1_motor_voltage = 	(short) round(arrayToFloat(&(pkt->data[4]))); //Motor voltage
			detailed_data.P1_motor_current = 	arrayToFloat(&(pkt->data[8])); //Motor current

			if (batteryState == FAULTED && fabs(detailed_data.P1_motor_current) > fabs(detailed_data.max_motor_current))
				detailed_data.max_motor_current = detailed_data.P1_motor_current;
		} else if (pkt->data[0] == MCMB_SUPP_BATT_VOLTAGE_ID){
			default_data.P2_low_supp_volt = (uint8_t) round(10.0 * arrayToFloat(&(pkt->data[4])));

		} else if (pkt->data[0] == MCMB_CAR_SPEED_ID){ //Car speed
			default_data.P1_speed_kph = pkt->data[1]; //Car speed (uint8_t)
		} else if (pkt->data[0] == MCMB_MOTOR_TEMPERATURE_ID) {
			detailed_data.P1_motor_temperature = (short) arrayToFloat(&(pkt->data[4]));

			if (detailed_data.P1_motor_temperature > 100)
				default_data.motor_warning = 1;
			else
				default_data.motor_warning = 0;
		}
		break;

	case BBMB_ID:
		 //Update connection status
		 BBMB_last_packet_tick_count = xTaskGetTickCount();
		 BBMBFirstPacketReceived = 1;
		 if (pkt->data[0] == BBMB_BUS_METRICS_ID){ //HV bus
			common_data.battery_power = 		(short) round(arrayToFloat(&(pkt->data[4])) * arrayToFloat(&(pkt->data[8]))); //Battery power
			detailed_data.P1_battery_voltage = 	(short) round(arrayToFloat(&(pkt->data[4]))); //Battery voltage
			detailed_data.P1_battery_current =  arrayToFloat(&(pkt->data[8])); //Battery current
			detailed_data.P2_HV_voltage = detailed_data.P1_battery_voltage;
			batteryVoltage = arrayToFloat(&(pkt->data[4])); //Battery voltage

			if (batteryState == FAULTED && fabs(detailed_data.P1_battery_current) > fabs(detailed_data.max_battery_current))
				detailed_data.max_battery_current = detailed_data.P1_battery_current;
		 } else if (pkt->data[0] == BBMB_LP_BUS_METRICS_ID){ //LV bus
			 common_data.LV_power = 			(short) round(10*arrayToFloat(&(pkt->data[4])) * arrayToFloat(&(pkt->data[8]))); //LV power
			 common_data.LV_voltage = 			(short) round(10*arrayToFloat(&(pkt->data[4]))); //LV voltage
			 detailed_data.P2_LV_current = 		(short) round(10*arrayToFloat(&(pkt->data[8]))); //LV current

		 } else if (pkt->data[0] == BBMB_RELAYS_STATE_ID){ //Relay state
		 	vTaskSuspendAll();
			batteryState = pkt->data[1];
			if (batteryState == HEALTHY && previousBatteryState == FAULTED) {
				battery_faulted_display_selection = 7;
			}
			previousBatteryState = batteryState;
			xTaskResumeAll();
			 batteryRelayState = pkt->data[2]; //Update global variable tracking battery relay state
			 common_data.battery_relay_state = batteryRelayState;
			 if (batteryRelayState == CLOSED) {
				 // turn on the back up camera and screen by regulation
				  HAL_GPIO_WritePin(GPIOI, BACKUP_CAMERA_CTRL_Pin, GPIO_PIN_SET); //Enable camera
				  HAL_GPIO_WritePin(GPIOI, BACKUP_SCREEN_CTRL_Pin, GPIO_PIN_SET); //Enable screen

			 } else if (batteryRelayState == OPEN) {
				 if (!camera_switch_is_on) {
					 // turn off back up camera and screen
					  HAL_GPIO_WritePin(GPIOI, BACKUP_CAMERA_CTRL_Pin, GPIO_PIN_RESET); //Disable camera
					  HAL_GPIO_WritePin(GPIOI, BACKUP_SCREEN_CTRL_Pin, GPIO_PIN_RESET); //Disable screen

				 }
			 }
			 //Reset VFM (when motor controller loses power, upon startup, VFM resets so we want the display to match)
			 if (batteryRelayState == OPEN) default_data.P2_VFM = 0;

			 detailed_data.faultType = pkt->data[4];
			 detailed_data.faultCell = pkt->data[5];
			 detailed_data.faultTherm = pkt->data[6];

		 } else if (pkt->data[0] == BMS_CELL_TEMP_ID){ //Cell temperature
			 if (pkt->data[1] == 5){	BMS_last_packet_tick_count = xTaskGetTickCount();	} //Hearing from BMS #5 implies as the other ones are connected
														  //
			detailed_data.min_temperature = 10000;
			detailed_data.min_temperature_cell = 0;
			detailed_data.max_temperature = -10000;
			detailed_data.max_temperature_cell = 0;

			//Update global list
			for (int i = 0; i < NUM_TEMP_SENSORS_PER_MODULE; i++){
				uint8_t j = pkt->data[1] * NUM_TEMP_SENSORS_PER_MODULE + i;
				if (j < NUM_BATT_TEMP_SENSORS){ //Check that we're not writing outside of array bounds (in case of bad packet)
					float temp = arrayToFloat( &(pkt->data[4 * (i + 1)]) );
					battery_temp[j] = temp;
//					detailed_data.overtemperature_status -= detailed_data.overtemperature_status & (1 << j);
//					detailed_data.undertemperature_status -= detailed_data.undertemperature_status & (1 << j);
					int32_t scaled_temperature = temp * 10;
					if (temp != BATTERY_TEMPERATURES_INITIAL_VALUE) {
						if (scaled_temperature < detailed_data.min_temperature) {
							detailed_data.min_temperature = scaled_temperature;
							detailed_data.min_temperature_cell = j;
						}

						if (scaled_temperature > detailed_data.max_temperature) {
							detailed_data.max_temperature = scaled_temperature;
							detailed_data.max_temperature_cell = j;
						}
					}


					if (temp != BATTERY_TEMPERATURES_INITIAL_VALUE) {
						if (temp > HV_BATT_OT_THRESHOLD) {
							detailed_data.overtemperature_status |= 1 << j;
						} else if (temp < HV_BATT_UT_THRESHOLD) {
							detailed_data.undertemperature_status |= 1 << j;
						}
					}

					if ((detailed_data.overtemperature_status & (1 << j)) && temp > detailed_data.max_cell_temperatures[j]) {
						detailed_data.max_cell_temperatures[j] = temp;
					}

					if ((detailed_data.undertemperature_status & (1 << j)) && temp < detailed_data.min_cell_temperatures[j]) {
						detailed_data.min_cell_temperatures[j] = temp;
					}
				}
			}

//			//Update display with new max. temp
//			float max_temp = -100; //Low initial value to find max. temp
//			for (int i = 0; i < NUM_BATT_TEMP_SENSORS; i++){
//				if (battery_temp[i] > max_temp){	max_temp = battery_temp[i];	}
//			}
//			detailed_data.P2_max_batt_temp = (short) 10.0*max_temp;

		 } else if (pkt->data[0] == BMS_CELL_VOLT_ID){ //Cell voltage
			 if (pkt->data[1] == NUM_BMS_MODULES-1){	BMS_last_packet_tick_count = xTaskGetTickCount();	} //Hearing from BMS #5 implies as the other ones are connected

			detailed_data.min_voltage = 10000;
			detailed_data.min_voltage_cell = 0;
			detailed_data.max_voltage = -10000;
			detailed_data.max_voltage_cell = 0;

			//Update global list
			// TODO: get highest and lowest cell voltage and show on display
			for (int i = 0; i < NUM_CELLS_PER_MODULE; i++){
				uint8_t j = pkt->data[1]*NUM_CELLS_PER_MODULE + i;
				if (j < NUM_BATT_CELLS) {
					float voltage = arrayToFloat( &(pkt->data[4 * (i + 1)]) );
					battery_cell_voltages[j] = voltage;
					int32_t scaled_voltage = voltage * 10;
					if (voltage != BATTERY_CELL_VOLTAGES_INITIAL_VALUE && voltage != BATTERY_CELL_VOLTAGES_FAKE_VALUE) {
						if (scaled_voltage < detailed_data.min_voltage) {
							detailed_data.min_voltage = scaled_voltage;
							detailed_data.min_voltage_cell = j;
						}

						if (scaled_voltage > detailed_data.max_voltage) {
							detailed_data.max_voltage = scaled_voltage;
							detailed_data.max_voltage_cell = j;
						}
					}


//					detailed_data.overvoltage_status -= detailed_data.overvoltage_status & (1 << j);
//					detailed_data.undervoltage_status -= detailed_data.undervoltage_status & (1 << j);


					if (voltage != BATTERY_CELL_VOLTAGES_INITIAL_VALUE && voltage != BATTERY_CELL_VOLTAGES_FAKE_VALUE) {
						if (voltage > HV_BATT_OV_THRESHOLD) {
							detailed_data.overvoltage_status |= 1 << j;
						} else if (voltage < HV_BATT_UV_THRESHOLD) {
						  detailed_data.undervoltage_status |= 1 << j;
						}
					}

					if ((detailed_data.overvoltage_status & (1 << j)) && voltage > detailed_data.max_cell_voltages[j]) {
						detailed_data.max_cell_voltages[j] = voltage;
					}

					if ((detailed_data.undervoltage_status & (1 << j)) && voltage < detailed_data.min_cell_voltages[j]) {
						detailed_data.min_cell_voltages[j] = voltage;
					}
				}
			}

		 } else if (pkt->data[0] == BMS_CELL_SOC_ID){ //Cell state of charge
			 if (pkt->data[1] == 5){	BMS_last_packet_tick_count = xTaskGetTickCount();	} //Hearing from BMS #5 implies as the other ones are connected

			detailed_data.min_soc = 100;
			detailed_data.min_soc_cell = 0;
			detailed_data.max_soc = 0;
			detailed_data.max_soc_cell = 0;

			//Update global list
			for (int i = 0; i < NUM_CELLS_PER_MODULE; i++){
				uint8_t j = pkt->data[1]*NUM_CELLS_PER_MODULE + i;
				if (j < NUM_BATT_CELLS) {
					float soc = arrayToFloat( &(pkt->data[4 * (i + 1)]) );
					battery_soc[j] = soc;
					int32_t scaled_soc = soc * 100;
					if (soc != BATTERY_SOC_INITIAL_VALUE) {
						if (scaled_soc < detailed_data.min_soc) {
							detailed_data.min_soc = scaled_soc;
							detailed_data.min_soc_cell = j;
						}

						if (scaled_soc > detailed_data.max_soc) {
							detailed_data.max_soc = scaled_soc;
							detailed_data.max_soc_cell = j;
						}
					}

				}
			}

			 //Update display with new min. soc
			 float min_soc = 200; //High initial value to find min. soc
			 float total_soc = 0;
			 for (int i = 0; i < NUM_BATT_CELLS; i++){

				//if (battery_soc[i] < min_soc){	min_soc = battery_soc[i];	}
				 if (battery_soc[i] != BATTERY_SOC_INITIAL_VALUE) {
					 total_soc += battery_soc[i];
				 }
			 }
			 //common_data.battery_soc = (short) 100*min_soc;
			 common_data.battery_soc = total_soc / (short)NUM_BATT_CELLS * 100;
		 }
		 break;

	case CHASE_ID:
		 if (pkt->data[0] == CHASE_HEARTBEAT_ID){
			 //Update connection status
			 Chase_last_packet_tick_count = xTaskGetTickCount();
		 }

		 break;
	}
	xTaskResumeAll();
}

void steeringWheelTask(const void *pv){
// {0xa5, 0x03, DATA_1, DATA_2, DATA_3, steering_wheel_variable_regen_value, CRC}

// DATA_0: [ACC8, ACC7, ACC6, ACC5, ACC4, ACC3, ACC2, ACC1] <-- ROTARY ENCODER DATA
// DATA_1: [x, x, x, CRUISE, HORN, RADIO, RIGHT_INDICATOR, LEFT_INDICATOR]
// DATA_2  : [x, x, x, SELECT, RIGHT, LEFT, DOWN, UP]

	uint8_t oldSteeringData[3] = {0, 0, 0};
	uint8_t emergencyLight = 0;

	uint8_t expectedLen = 7; //must be same length as sent from SWB
	uint8_t rxBuf[expectedLen];
	for(;;){
	  	B_uartReadFullMessage(swBuart,  rxBuf,  expectedLen, BSSR_SERIAL_START);

	  	vTaskSuspendAll();
		//Save old data
		for (int i = 0; i < 3; i++){ oldSteeringData[i] = steeringData[i]; }

		//Update global data, and check value after start byte for integrity
		if (rxBuf[0] == BSSR_SERIAL_START && rxBuf[1] == 0x03){
			for (int i = 0; i < 3; i++){
				if (steeringData[i] != rxBuf[2 + i]){
					steeringData[i] = rxBuf[2 + i]; } //Only update if different (it should be different)
			}
#ifdef USE_ADC_REGEN
			//Update variable regen value
			steering_wheel_variable_regen_value = rxBuf[5];
#endif
		}
		xTaskResumeAll(); // exit critical section

		//------- Send acknowledge -------//
		//uint8_t buf_to_swb[2] = {BSSR_SERIAL_START, BSSR_SPB_SWB_ACK};
		//HAL_UART_Transmit(&huart8, buf_to_swb, sizeof(buf_to_swb), 100);

		// Only send info when state of steeringData changes to avoid bus traffic
		if (oldSteeringData[0] != steeringData[0] || oldSteeringData[1] != steeringData[1] || oldSteeringData[2] != steeringData[2]){

			//------- Send to RS485 bus -------//
			uint8_t buf_rs485[4] = {DCMB_STEERING_WHEEL_ID, steeringData[0], steeringData[1], steeringData[2]};
			B_tcpSend(btcp, buf_rs485, sizeof(buf_rs485));

			vTaskSuspendAll();
			//---------- Process data ----------//
			// Navigation <- Not implemented
\
			//INDICATOR LIGHTS - SEND TO BBMB
			//Left indicator - SEND TO BBMB
			uint8_t bufh1[2] = {DCMB_LIGHTCONTROL_ID, 0, 0, 0}; //[DATA ID, LIGHT INSTRUCTION]
			if (steeringData[1] & (1 << 0)){ //If LEFT_INDICATOR == 1 --> Extend lights (ON)
				bufh1[1] = 0b01000010;
				default_data.P1_left_indicator_status = 0;
			} else { //OFF
				bufh1[1] = 0b00000010;
				default_data.P1_left_indicator_status = 1;
			}
			xTaskResumeAll();
//			vTaskDelay(5);
			B_tcpSend(btcp, bufh1, sizeof(bufh1));
			vTaskSuspendAll();
			uint8_t bufh2[2] = {DCMB_LIGHTCONTROL_ID, 0, 0, 0}; //[DATA ID, LIGHT INSTRUCTION]
			//Right indicator - SEND TO BBMB
			if (steeringData[1] & (1 << 1)){ //If RIGHT_INDICATOR == 1 --> Extend lights (ON)
				bufh2[1] = 0b01000011;
				default_data.P2_right_indicator_status = 0;
			} else { //OFF
				bufh2[1] = 0b00000011;
				default_data.P2_right_indicator_status = 1;
			}
			xTaskResumeAll();
//			vTaskDelay(5);
			B_tcpSend(btcp, bufh2, sizeof(bufh2));
			vTaskSuspendAll();
			//Nothing to do for the horn as its state will be d by BBMB from buf_rs485

			//Cruise - (Try to change) Motor state and send to MCMB
			if (steeringData[1] & (1 << 4)){
				if (motorState != CRUISE){ //If pressed and not already in cruise (nor off nor standby), put in cruise
					if ((motorState != REGEN) && (motorState != OFF) && (motorState != STANDBY)){
						motorState = CRUISE; // change global motorState
						default_data.P2_motor_state = CRUISE;

						if (CRUISE_MODE == CONSTANT_SPEED) {
						  motorTargetSpeed = default_data.P1_speed_kph; //Just recycle this variable set in serialParse
						  motorTargetPower = 0; //MCMB will be doing its own speed control, don't send motorTargetPower commands
						}
					}

				} else if (motorState == CRUISE){ //If already in cruise
							motorState = STANDBY;
							default_data.P2_motor_state = STANDBY;
				  motorTargetSpeed = 0;
				}
			}

			//Radio - Enable driver voice radio
			if (steeringData[1] & (1 << 2)){
				//No plan to implement it in GEN11
			}

			//Up button pressed
			if (~oldUpButton && (steeringData[2] & (1 << 0))){ // 0 --> 1 transition
				if (default_data.P2_VFM < MAX_VFM - 1 && (batteryRelayState == CLOSED)){ //Bound VFM setting
					default_data.P2_VFM++;
				}
				vfmUpState = 1;

			} else if (oldUpButton && ~(steeringData[2] & (1 << 0))){ // 1 --> 0 transition
				vfmUpState = 0; //reset this in motorDataTimer
			}
			oldUpButton = (steeringData[2] & (1 << 0));

			//Down button pressed
			if (~oldDownButton && (steeringData[2] & (1 << 1))){ // 0 --> 1 transition
				if (default_data.P2_VFM > 0){ //Bound VFM setting
					default_data.P2_VFM--;
				}
				vfmDownState = 1;

			} else if (oldDownButton && ~(steeringData[2] & (1 << 1))){ // 1 --> 0 transition
				// nothing to do because vfmDownState = 0;  will be reset in motorDataTimer
			}
			oldDownButton = (steeringData[2] & (1 << 1));

#ifndef USE_ADC_REGEN
			//Middle button pressed - Holding it causes regen
			if (~oldMiddleButton && (steeringData[2] & (1 << 4))){ // 0 --> 1 transition
				// if (-REGEN_BATTERY_VOLTAGE_THRESHOLD <= batteryVoltage && batteryVoltage <= REGEN_BATTERY_VOLTAGE_THRESHOLD){
				if (-REGEN_BATTERY_CELL_VOLTAGE_THRESHOLD <= detailed_data.max_voltage / 10.0 && detailed_data.max_voltage / 10.0 <= REGEN_BATTERY_CELL_VOLTAGE_THRESHOLD) {
					start_steering_wheel_constant_regen = 1;
				}
			} else if (oldMiddleButton && ~(steeringData[2] & (1 << 4))){ // 1 --> 0 transition
				start_steering_wheel_constant_regen = 0;
			}
			oldMiddleButton = (steeringData[2] & (1 << 4));
#endif
			//Left button pressed
			if (~oldLeftButton && (steeringData[2] & (1 << 2))){ // 0 --> 1 transition
				//Toggle between default and detailed display frames
		//		if (display_selection == 0){ display_selection = 1;
		//		} else if (display_selection){ display_selection = 0;};
				// Toggle between default and and 2 details so 3 total
				if (batteryState != FAULTED) {
					display_selection = (display_selection + 1)%4;
				} else {
					vTaskSuspendAll();
					if (battery_faulted_display_selection == 7) {
						battery_faulted_display_selection = 6;
					} else {
						battery_faulted_display_selection = 7;
					}
					xTaskResumeAll();
				}
			} else if (oldLeftButton && ~(steeringData[2] & (1 << 2))){ // 1 --> 0 transition
			}
			oldLeftButton = (steeringData[2] & (1 << 2));

			//Right button pressed (use for emergency light)
			uint8_t bufe[] = {DCMB_LIGHTCONTROL_ID, 0, 0, 0}; //[DATA ID, LIGHT INSTRUCTION]
			if ((oldRightButton == 0) && (steeringData[2] & (1 << 3))){ // 0 --> 1 transition
				if (!emergencyLight) {
					bufe[1] = 0b01010000; // turn on hazard indicator
					default_data.P1_left_indicator_status = 0;
					default_data.P2_right_indicator_status = 0;
					emergencyLight = 1;
					default_data.hazard = 1;
				} else {
					bufe[1] = 0b00010000; // turn off hazard indicator
					default_data.P1_left_indicator_status = 1;
					default_data.P2_right_indicator_status = 1;
					emergencyLight = 0;
					default_data.hazard = 0;
				}
			}
			oldRightButton = (steeringData[2] & (1 << 3));
			xTaskResumeAll();
//			vTaskDelay(5);
			B_tcpSend(btcp, bufe, sizeof(bufe));
		}
	}
}

void sidePanelTask(const void *pv){
// {0xa5, 0x04, sidePanelData, CRC};
// sidePanelData is formatted as [IGNITION, CAMERA, FWD/REV, FAN, AUX2, AUX1, AUX0, ARRAY]
	uint8_t sidePanelData = 0;
	uint8_t firstTime = 1;
	uint8_t firstBatteryState = -1; //unknown at this moment. Will be set to OPEN, CLOSED
	uint8_t toggleIgnitionRequired = 0;
	uint8_t firstArrayState = -1; //unknown at this moment. Will be set to OPEN, CLOSED
	uint8_t toggleArrayRequired = 0;

	uint8_t expectedLen = 4; //must be same length as sent from SPB
    uint8_t rxBuf[expectedLen];

	for(;;){
		//e = B_uartRead(spbBuart);
		B_uartReadFullMessage(spbBuart,  rxBuf, expectedLen, BSSR_SERIAL_START);
		if (rxBuf[0] == BSSR_SERIAL_START && rxBuf[1] == 0x04){
			if (sidePanelData != rxBuf[2] || firstTime){
				sidePanelData = rxBuf[2];
				//Only update if different (it should be different)


			//------- Send acknowledge -------//
				uint8_t buf_to_spb[2] = {BSSR_SERIAL_START, BSSR_SPB_SWB_ACK};
				HAL_UART_Transmit(&huart3, buf_to_spb, sizeof(buf_to_spb), 100);

			//---------- Process data ----------//
			vTaskSuspendAll(); // data into global variable -> enter critical section
			//REAR CAMERA AND SCREEN
				if (sidePanelData & (1 << 6)){
				  HAL_GPIO_WritePin(GPIOI, BACKUP_CAMERA_CTRL_Pin, GPIO_PIN_SET); //Enable camera
				  HAL_GPIO_WritePin(GPIOI, BACKUP_SCREEN_CTRL_Pin, GPIO_PIN_SET); //Enable screen
				  camera_switch_is_on = 1;
				} else {
				  HAL_GPIO_WritePin(GPIOI, BACKUP_CAMERA_CTRL_Pin, GPIO_PIN_RESET); //Disable camera
				  HAL_GPIO_WritePin(GPIOI, BACKUP_SCREEN_CTRL_Pin, GPIO_PIN_RESET); //Disable screen
				  camera_switch_is_on = 0;
				}

			//FAN
				if (sidePanelData & (1 << 4)){
				  HAL_GPIO_WritePin(GPIOG, FAN_CTRL_Pin, GPIO_PIN_SET); //Enable fan
				} else {
				  HAL_GPIO_WritePin(GPIOG, FAN_CTRL_Pin, GPIO_PIN_RESET); //Disable fan
				}

				//AUX0 (DRL in GEN11) (Regulation requires DRL to turn on the moment battery relay is closed,
				// ...so when battery relay is closed, even if DRL switch is off, it will automatically turn on
				uint8_t bufh[4] = {DCMB_LIGHTCONTROL_ID, 0, 0, 0}; //[DATA ID, LIGHT INSTRUCTION]
				if (sidePanelData & (1 << 1)){
					bufh[1] = 0b01000100; //AUX0 == 1 -> DRL on
					default_data.P2_DRL_state = 1;
					default_data.light = 1;

				} else {
					bufh[1] = 0b00000100; //AUX0 == 0 -> DRL off
					default_data.P2_DRL_state = 0;
					default_data.light = 0;
				}
				xTaskResumeAll();
//				vTaskDelay(5);
				B_tcpSend(btcp, bufh, sizeof(bufh));
				vTaskSuspendAll();

			//AUX1 (display backlight control in GEN11)
				if (sidePanelData & (1 << 2)){
					HAL_GPIO_WritePin(DISP_LED_CTRL_GPIO_Port, DISP_LED_CTRL_Pin, GPIO_PIN_SET);
				} else {
					HAL_GPIO_WritePin(DISP_LED_CTRL_GPIO_Port, DISP_LED_CTRL_Pin, GPIO_PIN_RESET);
				}

			//AUX2 (ECO/PWR motor state control in GEN11)
				if (sidePanelData & (1 << 3)){
					ecoPwrState = 1; //1 is PWR
					default_data.eco = 1;
				} else {
					ecoPwrState = 0; //0 is ECO
					default_data.eco = 0;
				}

			//--- Update states ---//
			//ARRAY
				uint8_t bufh3[4] = {DCMB_RELAYS_STATE_ID, batteryState, batteryRelayState, arrayRelayState};
				// will not respond to toggle the first time this runs. Forces user to toggle for safety reasons.
				if (firstTime) {
					firstArrayState = (sidePanelData & (1 << 0))? OPEN : CLOSED;
					toggleArrayRequired = (firstArrayState == OPEN) ? 1 : 0;
				} else {
					if (sidePanelData & (1 << 0)){ //Array switch is ON (try to close relays)
						if (!toggleArrayRequired && batteryState != FAULTED){
							bufh3[3] = CLOSED;
						}
					} else { //Array switch is OFF (open relays)
						bufh3[3] = OPEN;
						toggleArrayRequired = 0;
					}
				}

			//FWD/REV (change motor direction forward or reverse and turn on displays if reverse)
				if (sidePanelData & (1 << 5)){ 	//Forward
				  fwdRevState = 0;
				  default_data.direction = FORWARD;
				} else {						//Reverse
				  fwdRevState = 1;
				  default_data.direction = REVERSE;
				}

			//IGNITION
				// will not respond to ignition the first time this runs. Forces user to toggle ignition for safety reasons.
				if (firstTime) {
					firstBatteryState = (sidePanelData & (1 << 7))? OPEN : CLOSED;
					toggleIgnitionRequired = (firstBatteryState == OPEN) ? 1 : 0;
				} else {
					if (sidePanelData & (1 << 7)){ //Ignition ON
						if (!toggleIgnitionRequired) {
							ignitionState = IGNITION_ON;
							bufh3[2] = CLOSED;
						}
					} else { //Ignition OFF
						ignitionState = IGNITION_OFF;
						default_data.P2_motor_state = OFF;
						bufh3[2] = OPEN;
						toggleIgnitionRequired = 0;
					}
				}
				xTaskResumeAll();
//				vTaskDelay(5);
				B_tcpSend(btcp, bufh3, sizeof(bufh3));
				firstTime = 0;
			}
		}

	}
}

void displayTask(const void *pv){
	uint8_t startupDone = 0;
	pToggle = 0;
	glcd_init();
	glcd_clear();

	default_data.P1_left_indicator_status = 1;
	default_data.P2_right_indicator_status = 1;
	default_data.batt_warning = 0;
	default_data.hazard = 0;

	/* Display selection (sel):
	 * 0: Default
	 * 1: Detailed
	 * 2: Cruise control activated
	 * 3: Cruise control deactivated
	 * 4: Ignition off (car is sleeping)
	 * 5: BMS fault
	 */
	while(1){
		if (startupDone == 0){
			drawLogo();
			osDelay(2000);
			startupDone = 1;
		} else {
			uint8_t local_display_sel;
			uint32_t tick_cnt;

			vTaskSuspendAll();
			tick_cnt = xTaskGetTickCount();

			//Update connection status indicator
			detailed_data.P2_BB = 	0;
			detailed_data.P2_MC = 	0;
			detailed_data.P2_BMS = 	0;
			detailed_data.P2_PPT = 	0;
			detailed_data.P2_RAD = 	0;

			if ((tick_cnt - PPTMB_last_packet_tick_count) < CONNECTION_EXPIRY_THRESHOLD){	detailed_data.P2_PPT 	= 1;	}
			if ((tick_cnt - BBMB_last_packet_tick_count) < CONNECTION_EXPIRY_THRESHOLD){	detailed_data.P2_BB 	= 1;	}
			if ((tick_cnt - MCMB_last_packet_tick_count) < CONNECTION_EXPIRY_THRESHOLD){	detailed_data.P2_MC 	= 1;	}
			if ((tick_cnt - BMS_last_packet_tick_count) < CONNECTION_EXPIRY_THRESHOLD){		detailed_data.P2_BMS 	= 1;	}
			if ((tick_cnt - Chase_last_packet_tick_count) < CONNECTION_EXPIRY_THRESHOLD){	detailed_data.P2_RAD 	= 1;	}

			local_display_sel = display_selection;

			//Check if need to display low supplemental battery voltage alert
			if (LOW_SUPP_VOLT_DISP_ALERT_EN){
				if (default_data.P2_low_supp_volt < 10.0 * LOW_SUPP_VOLT_THRESHOLD){
					local_display_sel = 8;
				}
			}

			//Check if we need to display the "Car is sleeping" frame when neither PPTMB nor BBMB relays are closed
			// if (SLEEP_FRAME_EN) {
			// 	if (IGNORE_PPTMB){
			// 		if (batteryRelayState == OPEN) { local_display_sel = 6; }
			// 	} else {
			// 		if ((batteryRelayState == OPEN) && (arrayRelayState == OPEN)) { local_display_sel = 6; }
			// 	}
			// }

			//Overwrite display state if battery fault
			if (batteryState == FAULTED){
				// Allows pressing steering wheel button to toggle between
				// ..."fault display" (7) and "car is sleeping display" (6)
				local_display_sel = battery_faulted_display_selection; // either 7 or 6
				default_data.P2_motor_state = OFF;
				default_data.batt_warning = 1;
			}

			// if (detailed_data.overvoltage_status || detailed_data.undervoltage_status
			// 		|| detailed_data.overtemperature_status || detailed_data.undertemperature_status
			// 		|| batteryState == FAULTED)
                        //        local_display_sel = battery_faulted_display_selection;

			xTaskResumeAll();
			drawP1(local_display_sel);
			drawP2(local_display_sel);
			osDelay(500);
		}
	}
}

void motorDataTimer(TimerHandle_t xTimer){
	// Just in case pedalTask stops running (for safety)
	if (ignitionState != IGNITION_ON){ //overrides original motor state if IGNITION is not on
		motorState = OFF;
		motorTargetPower = (uint16_t) 0;
    motorTargetSpeed = 0;
		default_data.P2_motor_state = OFF;
	}

	static uint8_t buf[12] = {0};

	// -------- BUTTONS -------- //
	uint8_t digitalButtons = 0;
	digitalButtons |= fwdRevState << 3;
	digitalButtons |= vfmUpState << 2;
	digitalButtons |= vfmDownState << 1;
	digitalButtons |= ecoPwrState;

	if(vfmUpState == 1){
		vfmUpState = 0;
	}
	if(vfmDownState == 1){
		vfmDownState = 0;
	}

	// -------- SEND -------- //
	buf[0] = DCMB_MOTOR_CONTROL_STATE_ID;
	buf[1] = motorState;
	buf[2] = digitalButtons;
	buf[3] = default_data.P2_VFM;
	packi16(&buf[4], (uint16_t) motorTargetPower);
	buf[8] = motorTargetSpeed;

	B_tcpSend(btcp, buf, sizeof(buf));
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
  else if (htim->Instance == TIM7) {
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
