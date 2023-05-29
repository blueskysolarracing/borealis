/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "h7Boot.h"
#include "buart.h"
#include "btcp.h"
#include "psm.h"
#include "protocol_ids.h"
#include "math.h"
#include "mitsuba_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR 16
#define FWD_REV 8
#define ECO_PWR 1
#define VFM_UP 4
#define VFM_DOWN 2

#define WHEEL_DIA 0.555625 //In meters

#define HEARTBEAT_INTERVAL 1000 //Period between heartbeats

//--- MOTOR ---//
enum CRUISE_MODE {
	CONSTANT_POWER,
	CONSTANT_SPEED
};

#define CRUISE_MODE CONSTANT_POWER //Specifies how how cruise control should work (maintains constant motorTargetPower or maintains motorTargetSpeed)
/* ^ Need to update in MCMB as well ^ */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
//--- PSM ---//
struct PSM_Peripheral psmPeriph;
double voltageCurrent_Motor[2] = {0, 0};
double voltageCurrent_Supp[2] = {0, 0};

struct PSM_FIR_Filter psmFilter;
float PSM_FIR_HV_Voltage[PSM_FIR_FILTER_SAMPLING_FREQ_MCMB] = {0};
float PSM_FIR_HV_Current[PSM_FIR_FILTER_SAMPLING_FREQ_MCMB] = {0};

//--- COMMS ---//
B_uartHandle_t *buart;
B_uartHandle_t *radioBuart;
B_tcpHandle_t *btcp;
B_tcpHandle_t *radioBtcp;

uint8_t heartbeat[2] = {MCMB_HEARTBEAT_ID, 0};

//uint16_t accValue = 0;
//uint16_t regenValue = 0;

typedef enum {
	OFF,
	PEDAL,
	CRUISE,
	REGEN,
	STANDBY
} MOTORSTATE;

enum motorPowerState {
	ECO,
	POWER
};

MOTORSTATE motorState; //see below for description
//	[0] OFF (not reacting to any input)
//	[1] PEDAL (motor power controlled by accelerator pedal)
//	[2] CRUISE (DCMB sends target speed to MCMB, which runs a control loop to maintain target speed)
//	[3] REGEN (When regen pedal is pressed; has priority over others). DCMB controls motor state
uint16_t targetPower = 0; // Note: this is not in watts, it is from 0 - 256, for POT
uint8_t targetSpeed = 0;
float batteryVoltage = 0;

// These are values from 5 digital buttons
//uint8_t motorOnOff = 0; // 1 means motor is on, 0 means off (deprecated)
uint8_t ecoPwrState = 0; //0 is ECO, 1 is POWER (default to ECO)
uint8_t gearUp = 0;
uint8_t fwdRevState = 0;
uint8_t gearDown = 0;
uint8_t vfmResetState = 0;

long lastDcmbPacket = 0;
uint8_t temperature = 0;
uint8_t speedTarget;
uint8_t globalKmPerHour = 0;

MotorInterface* motor;
MitsubaMotor mitsuba;

// Struct for reading PWM input (in this case: speed pulse from motor)
typedef struct {
	uint32_t icValue1;
	uint32_t icValue2;
	uint32_t diffCapture;
	uint16_t captureIndex;
	float frequency;
	uint32_t lastInterrupt;
} PWM_INPUT_CAPTURE;
PWM_INPUT_CAPTURE pwm_in = {0, 0, 1, 0, 0.0, 0};
// diffCapture must not be set to zero as it needs to be used as division and dividing by zero causes undefined behaviour

// Struct for cruise control variables -> PID controller
typedef struct{

	// Controller Gains
	float k_p;
	float k_i;
	float k_d;

	// Controller inputs
	float integrator;
	float prevError;
	float derivative;
	float prevMeasurement;

	// Controller output
	float output;

	// To set limits
	float outMin;
	float outMax;
	float integralMin;
	float integralMax;

	float time; // Sample time

} PIDController;
PIDController pid = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1};
// time must not be set to zero as it needs to be used as division
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM12_Init(void);
static void MX_WWDG_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* =================== Software Timers ========================*/
static void motorTmr(TimerHandle_t xTimer);
static void spdTmr(TimerHandle_t xTimer);
void HeartbeatHandler(TimerHandle_t xTimer);
/* ============================================================*/

//Cruise control task
void cruiseControlTaskHandler(void* parameters);

//Tasks for temperature reading and PSM and heartbeat
void tempSenseTaskHandler(void* parameters);
void PSMTaskHandler(void* parameters);
void measurementSender(TimerHandle_t xTimer);

// function which writes to the MCP4146 potentiometer on the MC^2
void MCP4161_Pot_Write(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr);

// Reads ADC by Polling. Note: the polling occurs in a separate thread to prevent blocking CPU.
uint16_t ADC_poll_read(ADC_HandleTypeDef *hadcPtr);

/*========== Helper functions for ADC and temperature reading ========== */
float ADCMapToVolt(float ADCValue);
float convertToTemp(float Vadc);
float getTemperature(ADC_HandleTypeDef *hadcPtr);
/*================================================================*/

/*========== Helper functions for Cruise Control Implementation ========== */
float PIDControllerUpdate(float setpoint, float measured);
float speedToFrequency(uint8_t targetSpeed);

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM12_Init();
  MX_WWDG_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	//--- NUKE LED ---//
	HAL_TIM_Base_Start_IT(&htim7);

	uint8_t SPI_START_VAL = 0b00010001;

    //---- COMMS ---//
    buart = B_uartStart(&huart4); //Use huart2 for uart test. Use huart4 for RS485
    btcp = B_tcpStart(MCMB_ID, &buart, buart, 1, &hcrc);

    //--- MOTOR ---//
    mitsuba.mainPort = MotorMain_GPIO_Port;
    mitsuba.mainPin = MotorMain_Pin;

    mitsuba.fwdRevPort = MotorFwdRev_GPIO_Port;
    mitsuba.fwdRevPin = MotorFwdRev_Pin;

    mitsuba.vfmUpPort = MotorVfmUp_GPIO_Port;
    mitsuba.vfmUpPin = MotorVfmUp_Pin;

    mitsuba.vfmDownPort = MotorVfmDown_GPIO_Port;
    mitsuba.vfmDownPin = MotorVfmDown_Pin;

    mitsuba.ecoPort = MotorEco_GPIO_Port;
    mitsuba.ecoPin = MotorEco_Pin;

    mitsuba.vfmResetPort = MotorVfmReset_GPIO_Port;
    mitsuba.vfmResetPin = MotorVfmReset_Pin;

    mitsuba.MT3Port = MotorMT3_GPIO_Port;
    mitsuba.MT3Pin = MotorMT3_Pin;

    mitsuba.MT2Port = MotorMT2_GPIO_Port;
    mitsuba.MT2Pin = MotorMT2_Pin;

    mitsuba.MT1Port = MotorMT1_GPIO_Port;
    mitsuba.MT1Pin = MotorMT1_Pin;

    mitsuba.MT0Port = MotorMT0_GPIO_Port;
    mitsuba.MT0Pin = MotorMT0_Pin;

    mitsuba.cs0AccelPort = MotorCSAccel_GPIO_Port;
    mitsuba.cs0AccelPin = MotorCSAccel_Pin;
    mitsuba.cs1RegenPort = MotorCSRegen_GPIO_Port;
    mitsuba.cs1RegenPin = MotorCSRegen_Pin;
    mitsuba.potSpiPtr = &hspi3;

    motor = mitsubaMotor_init(&mitsuba);

  //  //Gen11 regen write below:
  //  MCP4161_Pot_Write(0, MotorCSRegen_GPIO_Port, MotorCSRegen_Pin, &hspi3);
  //  MCP4161_Pot_Write(255, MotorCSRegen_GPIO_Port, MotorCSRegen_Pin, &hspi3);
  //  MCP4161_Pot_Write(0, MotorCSRegen_GPIO_Port, MotorCSRegen_Pin, &hspi3);
  //
  //  //Gen11 accel write below:
  //  MCP4161_Pot_Write(0, MotorCSAccel_GPIO_Port, MotorCSAccel_Pin, &hspi3);
  //  MCP4161_Pot_Write(255, MotorCSAccel_GPIO_Port, MotorCSAccel_Pin, &hspi3);
  //  MCP4161_Pot_Write(0, MotorCSAccel_GPIO_Port, MotorCSAccel_Pin, &hspi3);

    //--- PSM ---//
    psmPeriph.CSPin0 = PSM_CS_0_Pin;
    psmPeriph.CSPin1 = PSM_CS_1_Pin;
    psmPeriph.CSPin2 = PSM_CS_2_Pin;
    psmPeriph.CSPin3 = PSM_CS_3_Pin;

    psmPeriph.CSPort0 = PSM_CS_0_GPIO_Port;
    psmPeriph.CSPort1 = PSM_CS_1_GPIO_Port;
    psmPeriph.CSPort2 = PSM_CS_2_GPIO_Port;
    psmPeriph.CSPort3 = PSM_CS_3_GPIO_Port;

    psmPeriph.LVDSPin = PSM_LVDS_EN_Pin;
    psmPeriph.LVDSPort = PSM_LVDS_EN_GPIO_Port;

    psmPeriph.DreadyPin = PSM_DReady_Pin;
    psmPeriph.DreadyPort = PSM_DReady_GPIO_Port;

    PSM_Init(&psmPeriph, 2); //2nd argument is PSM ID (2 for MCMB)
    PSM_FIR_Init(&psmFilter); //Initialize FIR averaging filter for PSM
    psmFilter.buf_current = PSM_FIR_HV_Current;
    psmFilter.buf_voltage = PSM_FIR_HV_Voltage;
    psmFilter.buf_size = PSM_FIR_FILTER_SAMPLING_FREQ_MCMB;

    if (configPSM(&psmPeriph, &hspi2, &huart2, "12", 2000) == -1){ //2000ms timeout
  	  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //Turn on red LED as a warning
    }

    //--- FREERTOS ---//
    xTimerStart(xTimerCreate("motorStateTimer", 20, pdTRUE, NULL, motorTmr), 0);
    xTimerStart(xTimerCreate("spdTimer", 100, pdTRUE, NULL, spdTmr), 0);
    xTimerStart(xTimerCreate("HeartbeatHandler",  pdMS_TO_TICKS(HEARTBEAT_INTERVAL / 2), pdTRUE, (void *)0, HeartbeatHandler), 0); //Heartbeat handler
    //configASSERT(xTimerStart(xTimerCreate("PSMTaskHandler",  pdMS_TO_TICKS(round(1000 / PSM_FIR_FILTER_SAMPLING_FREQ_MCMB)), pdTRUE, (void *)0, PSMTaskHandler), 0)); //Temperature and voltage measurements
    configASSERT(xTimerStart(xTimerCreate("measurementSender",  pdMS_TO_TICKS(PSM_SEND_INTERVAL), pdTRUE, (void *)0, measurementSender), 0)); //Periodically send data on UART bus

    //HAL_TIM_Base_Start(&htim2); //not sure what this is for
    MX_TIM4_Init(); //CubeMX fails to generate this line, thus call manually
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);

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
#endif
  /* add threads, ... */
  BaseType_t status;
  // TaskHandle_t tempSense_handle;
  //
  //	status = xTaskCreate(tempSenseTaskHandler,  /* Function that implements the task. */
  //				"tempSenseTask", /* Text name for the task. */
  //				200, 		/* 200 words *4(bytes/word) = 800 bytes allocated for task's stack*/
  //				"none", /* Parameter passed into the task. */
  //				4, /* Priority at which the task is created. */
  //				&tempSense_handle /* Used to pass out the created task's handle. */
  //							  );
  //	configASSERT(status == pdPASS); // Error checking

  	TaskHandle_t PSM_handle;

  	status = xTaskCreate(PSMTaskHandler,  //Function that implements the task.
  				"PSMTask",  //Text name for the task.
  				200, 		 //200 words *4(bytes/word) = 800 bytes allocated for task's stack
  				"none",  //Parameter passed into the task.
  				4,  //Priority at which the task is created.
  				&PSM_handle  //Used to pass out the created task's handle.
  							);
  	configASSERT(status == pdPASS);// Error checking

  //	TaskHandle_t cruiseControl_handle;

  //	status = xTaskCreate(cruiseControlTaskHandler,  //Function that implements the task.
  //				"cruiseControlTask",  //Text name for the task.
  //				200, 		 //200 words *4(bytes/word) = 800 bytes allocated for task's stack
  //				"none",  //Parameter passed into the task.
  //				4,  //Priority at which the task is created.
  //				&cruiseControl_handle  //Used to pass out the created task's handle.
  //							);
  //	configASSERT(status == pdPASS);// Error checking

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
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
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MotorMT0_Pin|LED2_Pin|LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MotorMT1_Pin|MotorMT2_Pin|MotorMT3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|PSM_CS_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MotorVfmReset_Pin|MotorVfmDown_Pin|MotorVfmUp_Pin|MotorEco_Pin
                          |PSM_LVDS_EN_Pin|PSM_CS_1_Pin|PSM_CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MotorFwdRev_Pin|MotorMain_Pin|PSM_CS_3_Pin|PSM_DReady_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BBMB_PSM_CS_0_Pin|MotorCSAccel_Pin|MotorCSRegen_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MotorMT0_Pin LED2_Pin LED0_Pin */
  GPIO_InitStruct.Pin = MotorMT0_Pin|LED2_Pin|LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorMT1_Pin MotorMT2_Pin MotorMT3_Pin */
  GPIO_InitStruct.Pin = MotorMT1_Pin|MotorMT2_Pin|MotorMT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN0_Pin */
  GPIO_InitStruct.Pin = GPIO_IN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_IN0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN10_Pin */
  GPIO_InitStruct.Pin = GPIO_IN10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_IN10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin PSM_CS_0_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|PSM_CS_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorVfmReset_Pin MotorVfmDown_Pin MotorVfmUp_Pin MotorEco_Pin
                           PSM_LVDS_EN_Pin PSM_CS_1_Pin PSM_CS_2_Pin */
  GPIO_InitStruct.Pin = MotorVfmReset_Pin|MotorVfmDown_Pin|MotorVfmUp_Pin|MotorEco_Pin
                          |PSM_LVDS_EN_Pin|PSM_CS_1_Pin|PSM_CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorFwdRev_Pin MotorMain_Pin PSM_CS_3_Pin PSM_DReady_Pin */
  GPIO_InitStruct.Pin = MotorFwdRev_Pin|MotorMain_Pin|PSM_CS_3_Pin|PSM_DReady_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : BBMB_PSM_CS_0_Pin MotorCSAccel_Pin MotorCSRegen_Pin */
  GPIO_InitStruct.Pin = BBMB_PSM_CS_0_Pin|MotorCSAccel_Pin|MotorCSRegen_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * Function to set the wiper position of the MCP4146 potentiometer on the MC^2.
 * Pass in a wiperValue (range from 0 - 256) to set the wiper position of the potentiometer (which ranges from 0 - 256).
 * Must also pass in the appropriate GPIO port and pins for chip select and the address of SPI handle.
 *
 * Note: In cubeMx make sure SPI CLK is below 10Mhz (SPI CLK =
 * 		And, configure SPI to send MSB first, and send 8 bits at a time
*/
void MCP4161_Pot_Write(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr) {
	if (wiperValue > 256) {
		wiperValue = 256; // Since the highest wiperValue is 256
	}
	uint8_t ninethDataBit = (wiperValue >> 8) & 0b1;
	uint8_t potAddress = 0b0000;
	uint8_t writeCommand = 0b00;

	uint8_t commandByte  = (potAddress << 4) | (writeCommand << 2) | ninethDataBit;
	uint8_t dataByte = wiperValue & 0xFF;

	uint8_t fullCommand[2] = {commandByte, dataByte};

	// Transmit using SPI
	HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspiPtr, fullCommand, sizeof(fullCommand), 100);
	HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);
}

// Get ADC value by polling
uint16_t ADC_poll_read(ADC_HandleTypeDef *hadcPtr) {

	// enable ADC on appropriate channel
	HAL_ADC_Start(hadcPtr);
	// Poll ADC  & TimeOut = 1mSec
	HAL_ADC_PollForConversion(hadcPtr, 1);
	// Get ADC value
	uint16_t adcVal = HAL_ADC_GetValue(hadcPtr);

	// turn off ADC
	HAL_ADC_Stop(hadcPtr);

	return adcVal;
}

// This function maps the ADC value to the actual ADC input voltage
float ADCMapToVolt(float ADCValue) {
	float ADCResolution = 4096.0; //ADC resolution should be 2^12 = 4096
	float ADCRefVoltage = 3.3;
	return ADCValue / ADCResolution * ADCRefVoltage;
}

//Converts ADC input voltage to its corresponding temperature using LMT86's data-sheet equation
float convertToTemp(float Vadc) {
	// change Voltage unit from Volts to miliVolts
	float Vadc_mV = Vadc *1000;

	// use equation from PG 9 of the LMT86 temperature sensor data-sheet
	float temperature = (10.888 - sqrtf(10.888*10.888 + 4*0.00347*(1777.3-Vadc_mV)))/(2*(-0.00347)) + 30;

	return temperature;
}

//Function to call to get the temperature measured by the tempSensor
float getTemperature(ADC_HandleTypeDef *hadcPtr) {
	float Vadc = ADCMapToVolt(ADC_poll_read(hadcPtr));
	float temperature = convertToTemp(Vadc);
	return temperature;
}

float speedToFrequency(uint8_t targetSpeed){

	float frequency = targetSpeed * 16 / (3600 * 1000 * 1.7156);
	// 1.7156 is the circumference of the wheel
	// This is assuming targetSpeed will be given in kmPerHour
	return frequency;

}

// Function to implement PID controller for cruise control
//float PIDControllerUpdate(float setpoint, float measured){
//
//	float error = setpoint - measured;
//
//	// Proportional term
//	float proportional = pid->k_p * error;
//	// Integral term
//	pid->integrator = pid->integrator + pid->k_i * pid->time * (error + pid->prevError)/2.0;
//
//	// Calculate integral limits
//	if(pid->outMax > proportional){
//		integralMax = pid->outMax - proportional;
//	} else {
//		integralMax = 0.0;
//	}
//
//	if(pid->outMax < proportional){
//		integralMin = pid->outMax - proportional;
//	} else {
//		integralMin = 0.0;
//	}
//
//	// Set limits to integration - integral anti-windup
//	if(pid->integrator > pid->integralMax){
//		pid->integrator = pid->integralMax;
//	} else if(pid->integrator < pid->integralMin){
//		pid->integrator = pid->integralMin;
//	}
//
//	// Derivative term
//	pid->derivative = pid->k_d * (error - pid->prevError) / pid->time;
//
//	// This one includes a filter to prevent HF amplification and on measurement to prevent derivative kick -> use if needed, but need tau term
//	//pid->derivative = -(2 * pid->k_d * (measurement - pid->prevMeasurement) + (2 * pid->tau - pid->time) * pid->derivative)/ (2* pid->tau + pid->time);
//
//	// Output
//	pid->output = proportional + pid->integrator + pid->derivative;
//
//	if(pid->output > pid->outMax){
//		pid->output = pid->outMax;
//	} else if(pid->output < pid->outMin){
//		pid->output = pid->outMin;
//	}
//
//	pid->prevError = error;
//	pid->prevMeasurement = measured;
//
//	return pid->output;
//}


static void motorTmr(TimerHandle_t xTimer){
	int res = -1; //res will be used for debugging

	//TODO: use software timer to handle tickcount overflow
	if(xTaskGetTickCount() >= (lastDcmbPacket + 4000)){  //if serialParse stops being called (this means uart connection is lost)

		motor->turnOff(motor);
		gearUp = 0;
		gearDown = 0;
		if (motor->isAccel(motor)) {
			res = motor->setAccel(motor, 0); // turns off accel
		}
		if (motor->isRegen(motor)) {
			res = motor->setRegen(motor, 0); // turns off regen
		}
		return;
	}

	switch (motorState) {
		case OFF:
			motor->turnOff(motor);
			if (motor->isAccel(motor)) {
				res = motor->setAccel(motor, 0); // turns off accel
			}
			if (motor->isRegen(motor)) {
				res = motor->setRegen(motor, 0); // turns off regen
			}
			return; //return instead of break here, since no need for VFM gear change
		case STANDBY:
			if (!motor->isOn(motor)) {
				res = motor->turnOn(motor);
			}
			if (motor->isOn(motor)) {
				if (fwdRevState) {
					res = motor->setReverse(motor);
				} else {
					res = motor->setForward(motor);
				}

				if (motor->isAccel(motor)) {
					res = motor->setAccel(motor, 0); // turns off accel
				}
				if (motor->isRegen(motor)) {
					res = motor->setRegen(motor, 0); // turns off regen
				}
			}
			break;

		case PEDAL:
			if (!motor->isOn(motor)) {
				motor->turnOn(motor);
			}
			if (motor->isOn(motor)) {
				if (fwdRevState) {
					res = motor->setReverse(motor);
				} else {
					res = motor->setForward(motor);
				}
				if (motor->isRegen(motor)) {
					res = motor->setRegen(motor, 0);
				}
				res = motor->setAccel(motor, targetPower);
			}
			break;

		case CRUISE:
			if (!motor->isOn(motor)) {
				res = motor->turnOn(motor);
			}
			if (motor->isOn(motor)) {
				if (fwdRevState) {
					res = motor->setReverse(motor);
				} else {
					res = motor->setForward(motor);
				}

				//Update motor power
				if (CRUISE_MODE == CONSTANT_POWER){
					res = motor->setAccel(motor, targetPower);
				} else if (CRUISE_MODE == CONSTANT_SPEED){
					// TODO: call pid controller update
				}
			}

			break;

		case REGEN:
			if (!motor->isOn(motor)) {
				res = motor->turnOn(motor);
			}
			if (motor->isOn(motor)) {
				if (fwdRevState) {
					res = motor->setReverse(motor);
				} else {
					res = motor->setForward(motor);
				}
				if (motor->isAccel(motor)) {
					res = motor->setAccel(motor, 0);
				}
				res = motor->setRegen(motor, targetPower);
			}
			break;
	}
	//Set ECO/PWR mode and change gears
	if (motor->isOn(motor)) {
		if (ecoPwrState == ECO){
			res = motor->setEco(motor);
		} else if (ecoPwrState == POWER){
			res = motor->setPwr(motor);
		}
		if (gearUp && !gearDown) {
			res = motor->gearUp(motor);
			gearUp = 0;
		} else if (gearDown) {
			res = motor->gearDown(motor);
			gearDown = 0;
		}
	}
}

// New implementation GEN11
static void spdTmr(TimerHandle_t xTimer){
	/* Frequency computation */
	/* TIM5CLK = 1 MHz after prescalar is set to 75-1*/
	//Note 16 pulse (16 PWM periods) per wheel rotation

	if (xTaskGetTickCount() >= (pwm_in.lastInterrupt + pdMS_TO_TICKS(1000))){
		//Note: if 1 second passed and still no pwm interrupt, the car's wheel is turning once every 16 seconds or more
		//This is very slow and we will simply set frequency to zero to avoid diffCapture growing too large or even becoming infinite
		pwm_in.frequency = 0.0;
	}
	else {
		pwm_in.frequency = 1000000.0 / pwm_in.diffCapture;
	}
	//Note 16 pulse per rotation
	static uint8_t buf[4] = {MCMB_CAR_SPEED_ID, 0x00, 0x00, 0x00};

	// Can divide by 16 and multiply by 60 for Rotation per min

	// Get KM per Hour
	float meterPerSecond = pwm_in.frequency / 16.0 * WHEEL_DIA * 3.14159;
	float kmPerHour = meterPerSecond / 1000.0 * 3600.0;

	// Send frequency to DCMB (for now)
	buf[1] = (uint8_t) round(kmPerHour);

	globalKmPerHour = kmPerHour; // used for debugger live expression

	B_tcpSend(btcp, buf, 4);
}

void tempSenseTaskHandler(void* parameters) {
	static uint8_t buf[4] = {MCMB_MOTOR_TEMPERATURE_ID, 0x00, 0x00, 0x00};
	while(1) {
		temperature = (uint8_t)getTemperature(&hadc1);
		vTaskDelay(pdMS_TO_TICKS(1000));

		buf[1] = temperature;
//		B_tcpSend(btcp, buf, 4); // Temperature sense is not implemented hardware wise
	}
}



void cruiseControlTaskHandler(void* parameters){

//	while(1) {
//		float target = speedToFrequency(speedTarget);
//		pwm_in.frequency = PIDControllerUpdate(target, pwm_in.frequency);
//		// TODO: last line makes no sense
//		// TODO: might want to remove this task,might not exactly necessary
//	}

}


void serialParse(B_tcpPacket_t *pkt){
	switch(pkt->senderID){
	  case DCMB_ID:
		lastDcmbPacket = xTaskGetTickCount();
		if(pkt->data[0] == DCMB_MOTOR_CONTROL_STATE_ID){
			motorState = pkt->data[1];
			targetPower = unpacku16(&pkt->data[4]);
			targetSpeed = pkt->data[8];

			//for 5 digital buttons (4 now):
			// Deprecated: motorOnOff = pkt->data[10] & MOTOR; //Note MOTOR = 0b10000
			fwdRevState = pkt->data[2] & FWD_REV; //FWD_REV = 0b1000
			ecoPwrState = pkt->data[2] & ECO_PWR; //ECO_PWR = 0b0001
			gearUp = ((pkt->data[2] & VFM_UP) != 0) ? 1 : 0; //VFM_UP = 0b100
			gearDown = ((pkt->data[2] & VFM_DOWN) != 0) ? 1 : 0; //VFM_DOWN = 0b10

		}
		break;
	}

}

/** To read PWM diff capture from motor
  * @brief  Input capture callback in non blocking mode.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// the following will run if the handle is Timer 1 and channel 1 (the pwm input)
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (pwm_in.captureIndex == 0)
		{
			/* Get the 1st input capture value */
			pwm_in.icValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			pwm_in.captureIndex = 1;
		}
		else if (pwm_in.captureIndex == 1)
		{
			/* Get the 2nd input capture value */
			pwm_in.icValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			/* Capture computation */
			if (pwm_in.icValue2 > pwm_in.icValue1)
			{
				pwm_in.diffCapture = pwm_in.icValue2 - pwm_in.icValue1;
			}
			else if (pwm_in.icValue2 < pwm_in.icValue1)
			{
				/* 0xFFFFFFFF is max TIM5 CCRx register value */
				// Note TIM5 has counter period of 0xFFFFFFFF
				pwm_in.diffCapture = ((0xFFFFFFFF-pwm_in.icValue1) + pwm_in.icValue2) + 1;
				//Note the +1 is needed to include zero
			}
			else
			{
				/* If capture values are equal, we have reached the limit of
				 * frequency measures */
				//Error_Handler();
				pwm_in.diffCapture = 1; // Needed to avoid undefined behavior in frequency computation below
			}

			/* Frequency computation */
			//TIM2CLK is driven by APB1 which is 75MHz
			/* After prescalar of 75-1, TIM2CLK = 1 MHz */
			//pwm_in.frequency = 1000000.0 / pwm_in.diffCapture; // will compute this elsewhere

			pwm_in.captureIndex = 0;
		}
		pwm_in.lastInterrupt = xTaskGetTickCount();
	}
}
//
//void PSMTaskHandler(void* parameters){
//	uint8_t busMetrics[3 * 4] = {0};
//	uint8_t suppBatteryMetrics[2 * 4] = {0};
//	double voltageCurrent_Motor_local[2] = {0, 0};
//	double voltageCurrent_Supp_local[2] = {0, 0};
//
//	busMetrics[0] = MCMB_BUS_METRICS_ID;
//	suppBatteryMetrics[0] = MCMB_SUPP_BATT_VOLTAGE_ID;
//
//	while (1) {
//		//Motor on channel #1
//		PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 1, voltageCurrent_Motor_local, 2);
//		floatToArray((float) voltageCurrent_Motor_local[0], busMetrics + 4); // fills 4 - 7 of busMetrics
//		floatToArray((float) voltageCurrent_Motor_local[1], busMetrics + 8); // fills 8 - 11 of busMetrics
//
//		//Supplemental battery on channel #2
//		PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 2, voltageCurrent_Supp_local, 2);
//		floatToArray((float) voltageCurrent_Supp_local[0], suppBatteryMetrics + 4); // fills 4 - 7 of suppBatteryMetrics
//
//		//Send to bus
//		B_tcpSend(btcp, busMetrics, sizeof(busMetrics));
//		B_tcpSend(btcp, suppBatteryMetrics, sizeof(suppBatteryMetrics));
//
//		vTaskSuspendAll();
//		//Place in global variables
//		voltageCurrent_Motor[0] = voltageCurrent_Motor_local[0];
//		voltageCurrent_Motor[1] = voltageCurrent_Motor_local[1];
//		voltageCurrent_Supp[0] = voltageCurrent_Supp_local[0];
//		voltageCurrent_Supp[1] = voltageCurrent_Supp_local[1];
//		batteryVoltage = (float) voltageCurrent_Motor_local[0];
//		xTaskResumeAll();
//		vTaskDelay(1000);
//
//
//		PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 1, HV_data, 2);
//
//		vTaskSuspendAll();
//
//		psmFilter.push(&psmFilter, (float) HV_data[0], VOLTAGE);
//		psmFilter.push(&psmFilter, (float) -1.0*HV_data[1], CURRENT); //Invert current polarity as a possible current from PSM means the battery is discharging
//
//		xTaskResumeAll();
//	}
//}

void PSMTaskHandler(void* parameters){

	double HV_data[2];
	int delay = pdMS_TO_TICKS(round(1000 / PSM_FIR_FILTER_SAMPLING_FREQ_MCMB));
	while (1) {
		PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 1, HV_data, 2);

		vTaskSuspendAll();

		psmFilter.push(&psmFilter, (float) HV_data[0], VOLTAGE);
		psmFilter.push(&psmFilter, (float) -1.0*HV_data[1], CURRENT); //Invert current polarity as a possible current from PSM means the battery is discharging

		xTaskResumeAll();
		vTaskDelay(delay);
	}
}

void measurementSender(TimerHandle_t xTimer){
//Battery
	uint8_t busMetrics_HV[3 * 4] = {0};
	busMetrics_HV[0] = MCMB_BUS_METRICS_ID;

	//Get HV average
	vTaskSuspendAll();
	float HV_voltage = psmFilter.get_average(&psmFilter, VOLTAGE);
	float HV_current = psmFilter.get_average(&psmFilter, CURRENT);
	xTaskResumeAll();

	floatToArray(HV_voltage, busMetrics_HV + 4); // fills 4 - 7 of busMetrics
	floatToArray(HV_current, busMetrics_HV + 8); // fills 8 - 11 of busMetrics

	B_tcpSend(btcp, busMetrics_HV, sizeof(busMetrics_HV));


//Supp
//	double supp_data[2];
//	uint8_t busMetrics_LV[3 * 4] = {0};
//	busMetrics_LV[0] = MCMB_LP_BUS_METRICS_ID;
//
//	vTaskSuspendAll();
//	PSMRead(&psmPeriph, &hspi2, &huart2, 1, 2, 2, supp_data, 2);
//	xTaskResumeAll();
//
//	floatToArray((float) supp_data[0], busMetrics_LV + 4); // fills 4 - 7 of busMetrics
//	floatToArray((float) supp_data[1], busMetrics_LV + 8); // fills 16 - 19 of busMetrics
//
//	B_tcpSend(btcp, busMetrics_LV, sizeof(busMetrics_LV));
}

void HeartbeatHandler(TimerHandle_t xTimer){
	//Send periodic heartbeat so we know the board is still running
	B_tcpSend(btcp, heartbeat, sizeof(heartbeat));
	heartbeat[1] = ~heartbeat[1]; //Toggle for next time
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
