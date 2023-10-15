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
#include "stdio.h"
#include "h7Boot.h"
#include "buart.h"
#include "btcp.h"
#include "newpsm.h"
#include "protocol_ids.h"
#include "math.h"
#include "mitsuba_motor.h"
#include "cQueue.h"
#include "FLASH_SECTOR_H7.h"
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

#define MOTOR_STATE_TMR_PERIOD_MS 20 //Interval (in ms) at which the motor state timer runs (also determines the cruise control PI controller timestep)

#define CRUISE_PI_CHASE_CMD_PASSWORD 123 //16-bit unsigned (not used)

#define CRUISE_PI_FLASH_ADDRESS 0x081E0000

//--- MOTOR ---//
enum CRUISE_MODE {
	CONSTANT_POWER,
	CONSTANT_SPEED
};

#define CRUISE_CONTROL_SPEED_AVERAGING_COUNT 20

#define CRUISE_MODE CONSTANT_SPEED //Specifies how how cruise control should work (maintains constant motorTargetPower or maintains motorTargetSpeed)
/* ^ Need to update in MCMB as well ^ */

//--- TEMPERATURE SENSOR ---//

#define TEMP_AVG_SIZE 				20

// Parameters for Platinum 3850 ppm/K
#define RESISTANCE_ZERO_DEG			100
#define COEF_A 						(3.9083 * 0.001)
#define COEF_B						(-5.775 * 0.0000001)
#define COEF_C						(-4.183 * 0.000000000001)
#define PULL_DOWN_RESISTANCE		68000

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

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg1;

SRAM_HandleTypeDef hsram4;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

//--- PSM ---//
struct PSM_P psmPeriph;
//double voltageCurrent_Motor[2] = {0, 0};
//double voltageCurrent_Supp[2] = {0, 0};

struct PSM_FIR_Filter psmFilter;
float PSM_FIR_HV_Voltage[PSM_FIR_FILTER_SAMPLING_FREQ_MCMB] = {0};
float PSM_FIR_HV_Current[PSM_FIR_FILTER_SAMPLING_FREQ_MCMB_CURRENT] = {0};

//--- COMMS ---//
B_uartHandle_t *buart;
B_uartHandle_t *radioBuart;
B_tcpHandle_t *btcp;
B_tcpHandle_t *radioBtcp;

uint8_t heartbeat[2] = {MCMB_HEARTBEAT_ID, 0};

typedef enum {
	OFF,
	PEDAL,
	CRUISE,
	REGEN,
	STANDBY
} MOTORSTATE;

enum motorPowerState {
	ECO,
	POWER_STATE
};

MOTORSTATE motorState; //see below for description
//	[0] OFF (not reacting to any input)
//	[1] PEDAL (motor power controlled by accelOrRegenerator pedal)
//	[2] CRUISE (DCMB sends target speed to MCMB, which runs a control loop to maintain target speed). Note: It is also possible to set it to maintain constant power
//	[3] regenStrength (When regenStrength pedal is pressed; has priority over others). DCMB controls motor state
uint16_t targetPower = 0; // Note: this is not in watts, it is from 0 - 256, for POT
uint16_t targetRegenStrength = 0;
uint8_t targetSpeed = 0;
float batteryVoltage = 0;

// These are values from 5 digital buttons
//uint8_t motorOnOff = 0; // 1 means motor is on, 0 means off (deprecated)
uint8_t ecoPwrState = 0; //0 is ECO, 1 is POWER (default to ECO)
uint8_t gearUp = 0;
uint8_t fwdRevState = 0;
uint8_t gearDown = 0;
uint8_t vfm_pos = 0; // default to zero
uint8_t past_vfm_pos = 0;

long lastDcmbPacket = 0;
float temperature = 0;
uint8_t speedTarget;
float kmPerHour = 0;
float emaKmPerHour = 0;  // Exponential Moving Average of speed

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

// Struct for cruise control variables -> PI controller
typedef struct{
  // Controller Gains
	float k_p;
	float k_i;
  float k_d;

	// Controller inputs
	float integrator;
	float prevError;
	float prevMeasurement;

	// Controller output
	float output;

	// To set limits
	float outMin;
	float outMax;
	float integralMin;
	float integralMax;
	float derivativeMin;
	float derivativeMax;

	float timeStep_ms; // Sample time

} PIDController;

PIDController cruise_control_pi = {
  //Determined with basic model (PI_cruise_control_simulation.py) followed by empirical testing
  .k_p = 250.0,
  .k_i = 0.015,
  .k_d = 0.0, //Set to 0.0 to effectively change PID to PI (more stable but slower)
  
  .integralMin = -200.0,
  .integralMax = 200.0,
  .integrator = 0.0,

  .derivativeMin = -100.0,
  .derivativeMax = 100.0,

  .outMin = -255.0,
  .outMax = 255.0,
  .output = 0.0,

  .timeStep_ms = MOTOR_STATE_TMR_PERIOD_MS,
  .prevMeasurement = 0.0,
  .prevError = 0.0, 
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_RTC_Init(void);
static void MX_WWDG1_Init(void);
static void MX_FMC_Init(void);
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
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* =================== Software Timers ========================*/
static void motorTmr(TimerHandle_t xTimer);
static void spdTmr(TimerHandle_t xTimer);
void HeartbeatHandler(TimerHandle_t xTimer);
/* ============================================================*/

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

// Special filtering
typedef struct StaticFloatQueue {
	float vals[PSM_FIR_FILTER_SAMPLING_FREQ_MCMB_CURRENT];
	Queue_t q;
}StaticFloatQueue;

static void sfq_init(StaticFloatQueue* this)
{
	q_init_static(
			&this->q,
			sizeof(float),
			PSM_FIR_FILTER_SAMPLING_FREQ_MCMB_CURRENT,
			FIFO,
			true, // Overwrite old values (so we only need to call push() and no need to pop())
			this->vals,
			sizeof(this->vals)
		);
}
StaticFloatQueue current_queue;

static bool sfq_push(StaticFloatQueue* this, float val)
{
	return q_push(&this->q, &val);
}

static bool sfq_peek_idx(StaticFloatQueue* this, float* ret_val, int idx)
{
	return q_peekIdx(&this->q, ret_val, idx);
}

static float sfq_get_avg(StaticFloatQueue* this)
{
	float total_past_vals = 0;
	int num_past_vals = 0;
	for (int idx = 0; idx < PSM_FIR_FILTER_SAMPLING_FREQ_MCMB_CURRENT; idx++){
		float past_val;
		if (sfq_peek_idx(this, &past_val, idx)) { // evaluates to true if val is available
			num_past_vals++;
			total_past_vals += past_val;
		}
	}
	return total_past_vals / (float)num_past_vals;
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
  //arm_boot();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

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
  /* USER CODE BEGIN 2 */
  //--- NUKE LED ---//
  HAL_TIM_Base_Start_IT(&htim7);

  //uint8_t SPI_START_VAL = 0b00010001;

  //---- COMMS ---//
  buart = B_uartStart(&huart4); //Use huart2 for uart test. Use huart4 for RS485
  btcp = B_tcpStart(MCMB_ID, &buart, buart, 1, &hcrc);

  //--- MOTOR ---//
  mitsuba.mainPort = GPIOJ;
  mitsuba.mainPin = GPIO_PIN_5;

  mitsuba.fwdRevPort = GPIOG;
  mitsuba.fwdRevPin = GPIO_PIN_1;

  mitsuba.vfmUpPort = GPIOI;
  mitsuba.vfmUpPin = GPIO_PIN_15;

  mitsuba.vfmDownPort = GPIOI;
  mitsuba.vfmDownPin = GPIO_PIN_14;

  mitsuba.ecoPort = GPIOG;
  mitsuba.ecoPin = GPIO_PIN_0;

  mitsuba.vfmResetPort = GPIOI;
  mitsuba.vfmResetPin = GPIO_PIN_13;

  mitsuba.MT3Port = GPIOI;
  mitsuba.MT3Pin = GPIO_PIN_12;

  mitsuba.MT2Port = GPIOF;
  mitsuba.MT2Pin = GPIO_PIN_2;

  mitsuba.MT1Port = GPIOI;
  mitsuba.MT1Pin = GPIO_PIN_9;

  mitsuba.MT0Port = GPIOE;
  mitsuba.MT0Pin = GPIO_PIN_3;

  mitsuba.cs0accelOrRegenPort = GPIOK;
  mitsuba.cs0accelOrRegenPin = GPIO_PIN_2;
  mitsuba.cs1regenStrengthPort = GPIOG;
  mitsuba.cs1regenStrengthPin = GPIO_PIN_2;
  mitsuba.potSpiPtr = &hspi3;

  motor = mitsubaMotor_init(&mitsuba);

//  //Gen11 regenStrength write below:
//  MCP4161_Pot_Write(0, GPIOG, GPIO_PIN_2, &hspi3);
//  MCP4161_Pot_Write(255, GPIOG, GPIO_PIN_2, &hspi3);
//  MCP4161_Pot_Write(0, GPIOG, GPIO_PIN_2, &hspi3);
////
//  //Gen11 accelOrRegen write below:
//  MCP4161_Pot_Write(0, GPIOK, GPIO_PIN_2, &hspi3);
//  MCP4161_Pot_Write(255, GPIOK, GPIO_PIN_2, &hspi3);
//  MCP4161_Pot_Write(0, GPIOK, GPIO_PIN_2, &hspi3);

  //--- PSM ---//
  psmPeriph.CSPin = PSM_CS_0_Pin;
  psmPeriph.CSPort = PSM_CS_0_GPIO_Port;
  psmPeriph.LVDSPort = PSM_LVDS_EN_GPIO_Port;
  psmPeriph.LVDSPin = PSM_LVDS_EN_Pin;
  psmPeriph.motherboard = MCMB_PSM;


  PSM_init(&psmPeriph, &hspi2, &huart2);
  PSM_FIR_Init(&psmFilter);

//  configTriggerMode(&psmPeriph);
//  resetPSM(&psmPeriph);
  test_config(&psmPeriph, &hspi2, &huart2);

  psmFilter.buf_voltage = PSM_FIR_HV_Voltage;
  //psmFilter.buf_current = PSM_FIR_HV_Current; // discarded to use a different filter
  sfq_init(&current_queue);
  psmFilter.buf_size = PSM_FIR_FILTER_SAMPLING_FREQ_MCMB;

  // override default cruise_control_pi gains from flash if the values stored in flash are not 0 (not used for now. Need more testing for reliability)
//  	uint8_t rxBuf[16] = {0};
//	Flash_Read_Data(CRUISE_PI_FLASH_ADDRESS, (uint32_t*)rxBuf, 2);
//	float k_p = arrayToFloat(rxBuf);
//	float k_i = arrayToFloat(rxBuf + 4);
//	float k_d = arrayToFloat(rxBuf + 8);
//	if (k_p != 0.0) cruise_control_pi.k_p = k_p;
//	if (k_i != 0.0) cruise_control_pi.k_i = k_i;
//	if (k_d != 0.0) cruise_control_pi.k_d = k_d;



//  if (configPSM(&psmPeriph, &hspi2, &huart2, "12", 2000) == -1){ //2000ms timeout
//	  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //Turn on red LED as a warning
//  }

  //--- FREERTOS ---//
  xTimerStart(xTimerCreate("motorStateTimer", MOTOR_STATE_TMR_PERIOD_MS, pdTRUE, NULL, motorTmr), 0);
  xTimerStart(xTimerCreate("spdTimer", 100, pdTRUE, NULL, spdTmr), 0);
  xTimerStart(xTimerCreate("HeartbeatHandler",  pdMS_TO_TICKS(HEARTBEAT_INTERVAL / 2), pdTRUE, (void *)0, HeartbeatHandler), 0); //Heartbeat handler
  configASSERT(xTimerStart(xTimerCreate("measurementSender",  pdMS_TO_TICKS(PSM_SEND_INTERVAL), pdTRUE, (void *)0, measurementSender), 0)); //Periodically send data on UART bus

  //HAL_TIM_Base_Start(&htim2); //not sure what this is for
  MX_TIM5_Init(); //CubeMX fails to generate this line, thus call manually
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
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
  TaskHandle_t tempSense_handle;

	status = xTaskCreate(tempSenseTaskHandler,  /* Function that implements the task. */
				"tempSenseTask", /* Text name for the task. */
				200, 		/* 200 words *4(bytes/word) = 800 bytes allocated for task's stack*/
				"none", /* Parameter passed into the task. */
				4, /* Priority at which the task is created. */
				&tempSense_handle /* Used to pass out the created task's handle. */
							  );
	configASSERT(status == pdPASS); // Error checking

	TaskHandle_t PSM_handle;

	status = xTaskCreate(PSMTaskHandler,  //Function that implements the task.
				"PSMTask",  //Text name for the task.
				200, 		 //200 words *4(bytes/word) = 800 bytes allocated for task's stack
				"none",  //Parameter passed into the task.
				4,  //Priority at which the task is created.
				&PSM_handle  //Used to pass out the created task's handle.
							);
	configASSERT(status == pdPASS);// Error checking


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI|RCC_PERIPHCLK_SDMMC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  htim2.Init.Prescaler = 1-1;
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
  sConfigIC.ICFilter = 5;
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
  huart8.Init.BaudRate = 230400;
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
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|LED2_Pin|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|PSM_CS_0_Pin|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5|PSM_LVDS_EN_Pin|PSM_CS_1_Pin|PSM_CS_2_Pin
                          |PSM_CS_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BBMB_PSM_CS_0_GPIO_Port, BBMB_PSM_CS_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 LED2_Pin PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|LED2_Pin|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PI9 PI12 PI13 PI14
                           PI15 PSM_CS_0_Pin PI4 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|PSM_CS_0_Pin|GPIO_PIN_4|GPIO_PIN_7;
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

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
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

  /*Configure GPIO pins : PJ5 PSM_LVDS_EN_Pin PSM_CS_1_Pin PSM_CS_2_Pin
                           PSM_CS_3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|PSM_LVDS_EN_Pin|PSM_CS_1_Pin|PSM_CS_2_Pin
                          |PSM_CS_3_Pin;
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

  /*Configure GPIO pins : PB12 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG2_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BBMB_PSM_CS_0_Pin */
  GPIO_InitStruct.Pin = BBMB_PSM_CS_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BBMB_PSM_CS_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_IN10_Pin */
  GPIO_InitStruct.Pin = GPIO_IN10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_IN10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PJ10 PJ11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : PK0 PK1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PH14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_SPI6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG12 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PSM_DReady_Pin */
  GPIO_InitStruct.Pin = PSM_DReady_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PSM_DReady_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

float calculate_r_infinity(float R0, float B) {
	float T0 = 298.15;
    return R0 * exp(-B / T0);
}

float calculate_T(float R, float R0, float B) {
    float r_inf = calculate_r_infinity(R0, B);
    return B / log(R / r_inf);
}

//Function to call to get the temperature measured by the tempSensor
float getTemperature(ADC_HandleTypeDef *hadcPtr) {

	float adc_voltage = ADCMapToVolt(ADC_poll_read(hadcPtr));
	float resistance = PULL_DOWN_RESISTANCE * 3.316 / adc_voltage - PULL_DOWN_RESISTANCE;
	// float temperature = (-1*COEF_A + sqrtf(COEF_A*COEF_A - 4*COEF_B*(1 - resistance/RESISTANCE_ZERO_DEG))) / (2*COEF_B);
	
	// 100kΩ Glass NTC Thermistor RN3446
	float R0 = 100.0; // 100 kOhm at 25 deg C
    float B = 3950.0; // B constant, 3950 Kelvin

	float T = calculate_T(resistance, R0, B);
    
	return temperature;
}

// Function to implement PI controller for cruise control
float PIControllerUpdate(float setpoint, float measured){
  //Returns a float between -255 (full regenStrength) and 255 (full accelOrRegen) to modulate motor power to maintain zero error (desired speed)

	float error = setpoint - measured;
	// Proportional term
	float proportionalError = cruise_control_pi.k_p * error;
	
  // Integral term
  cruise_control_pi.integrator = cruise_control_pi.integrator + cruise_control_pi.k_i * cruise_control_pi.timeStep_ms * (error + cruise_control_pi.prevError)/2.0;
  if (cruise_control_pi.integrator > cruise_control_pi.integralMax){
    cruise_control_pi.integrator = cruise_control_pi.integralMax;
  } else if (cruise_control_pi.integrator < cruise_control_pi.integralMin){
    cruise_control_pi.integrator = cruise_control_pi.integralMin;
  }

  //Derivative term
  float derivative = 0.0;
  if (cruise_control_pi.timeStep_ms > 0.0){ //Prevent MCU from crashing
    derivative = cruise_control_pi.k_d * (error - cruise_control_pi.prevError) / cruise_control_pi.timeStep_ms;
    if (derivative < cruise_control_pi.derivativeMin){
      derivative = cruise_control_pi.derivativeMin;
    } else if (derivative > cruise_control_pi.derivativeMax){
      derivative = cruise_control_pi.derivativeMax;
    }
  }

	// Output
	cruise_control_pi.output = proportionalError + cruise_control_pi.integrator + derivative;

	if(cruise_control_pi.output > cruise_control_pi.outMax){
		cruise_control_pi.output = cruise_control_pi.outMax;
	} else if (cruise_control_pi.output < cruise_control_pi.outMin){
		cruise_control_pi.output = cruise_control_pi.outMin;
	}

	cruise_control_pi.prevError = error;
	cruise_control_pi.prevMeasurement = measured;

	return cruise_control_pi.output;
}

static void motorTmr(TimerHandle_t xTimer){
	int res = -1; //res will be used for debugging

	uint32_t tick_cnt = xTaskGetTickCount();
	uint32_t diff = 0;

	if (tick_cnt >= lastDcmbPacket) {
		diff = tick_cnt - lastDcmbPacket;
	} else {
		diff = ((0xFFFFFFFF - lastDcmbPacket)) + tick_cnt + 1;
	}

	if(diff > 4000){  //if serialParse stops being called after 4 seconds (this means uart connection is lost)

		motor->turnOff(motor);
		gearUp = 0;
		gearDown = 0;
		return;
	}

	switch (motorState) {
		case OFF:
			motor->turnOff(motor);
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
				motor->setZero(motor);
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
				  //TODO: Might need to add hysteresis to PI is it switches between regenStrength and accelOrRegen too quickly
				  float cruise_control_PI_output = PIControllerUpdate(targetSpeed, emaKmPerHour);
				  if (cruise_control_PI_output <= 0.0){
					res = motor->setAccel(motor, 0);
				  } else {
					res = motor->setAccel(motor, (uint32_t)cruise_control_PI_output);
				  }
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
				res = motor->setRegen(motor, /*targetPower*/ 0, targetRegenStrength);
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
		uint8_t current_vfm_pos = vfm_pos;
		if (current_vfm_pos > past_vfm_pos) {
			for (int i = 0; i < current_vfm_pos - past_vfm_pos; i++) {
				res = motor->gearUp(motor);
			}
		}
		if (current_vfm_pos < past_vfm_pos) {
			for (int i = 0; i < past_vfm_pos - current_vfm_pos; i++) {
				res = motor->gearDown(motor);
			}
		}
		past_vfm_pos = current_vfm_pos;

		// Deprecated
//		if (gearUp && !gearDown) {
//			res = motor->gearUp(motor);
//			gearUp = 0;
//		} else if (gearDown) {
//			res = motor->gearDown(motor);
//			gearDown = 0;
//		}
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
		//pwm_in.frequency = 1000000.0 / pwm_in.diffCapture;
		pwm_in.frequency = 75000000.0 / pwm_in.diffCapture;
	}
	//Note 16 pulse per rotation
	static uint8_t buf[4] = {MCMB_CAR_SPEED_ID, 0x00, 0x00, 0x00};

	// Can divide by 16 and multiply by 60 for Rotation per min

	// Get KM per Hour
	float meterPerSecond = pwm_in.frequency / 16.0 * WHEEL_DIA * 3.14159;
	kmPerHour = meterPerSecond / 1000.0 * 3600.0;

	const float alpha = 0.5; // 0.0 < alpha < 1.0. A smaller alpha will provide more smoothing, but react slower to changes.

	// Update the exponential moving average
    emaKmPerHour = alpha * kmPerHour + (1 - alpha) * emaKmPerHour;

    buf[1] = (uint8_t) round(emaKmPerHour);

	B_tcpSend(btcp, buf, 4);
}

void tempSenseTaskHandler(void* parameters) {
	uint8_t buf[8] = {0};
	while(1) {
		temperature = 0;
		for(int i = 0; i < TEMP_AVG_SIZE; i++){
			temperature += getTemperature(&hadc1);
		}
		temperature = temperature / TEMP_AVG_SIZE;

		vTaskDelay(pdMS_TO_TICKS(1000));
		buf[0] = MCMB_MOTOR_TEMPERATURE_ID;
		floatToArray(temperature, buf + 4);

		B_tcpSend(btcp, buf, sizeof(buf));
	}
}

void serialParse(B_tcpPacket_t *pkt){
  vTaskSuspendAll();
	switch(pkt->senderID){
	  case DCMB_ID:
		lastDcmbPacket = xTaskGetTickCount();
		if(pkt->data[0] == DCMB_MOTOR_CONTROL_STATE_ID){
			motorState = pkt->data[1];
			targetPower = unpacku16(&pkt->data[4]);
			targetRegenStrength = unpacku16(&pkt->data[6]);
			targetSpeed = pkt->data[8];
			vfm_pos = pkt->data[3];

			//for 5 digital buttons (4 now):
			// Deprecated: motorOnOff = pkt->data[10] & MOTOR; //Note MOTOR = 0b10000
			fwdRevState = pkt->data[2] & FWD_REV; //FWD_REV = 0b1000
			ecoPwrState = pkt->data[2] & ECO_PWR; //ECO_PWR = 0b0001
//			gearUp = ((pkt->data[2] & VFM_UP) != 0) ? 1 : 0; //VFM_UP = 0b100
//			gearDown = ((pkt->data[2] & VFM_DOWN) != 0) ? 1 : 0; //VFM_DOWN = 0b10

		}
		break;

    case CHASE_ID:
      if (pkt->data[0] == CHASE_CRUISE_PI_GAIN_ID){
        /*Data format:
        * [ID, 0, 0, UNUSED, 
           k_p[3], k_p[2], k_p[1], k_p[0], 
           k_i[3], k_i[2], k_i[1], k_i[0],
           k_d[3], k_d[2], k_d[1], k_d[0]]
        *
        * NOTE: k_p, k_i and k_d must be multiplied by 100000 before transmission
        */ 
		if (pkt->data[1] == 1) cruise_control_pi.k_p = (float) unpacku32(&pkt->data[4]) / 100000.0;
		if (pkt->data[2] == 1) cruise_control_pi.k_i = (float) unpacku32(&pkt->data[8]) / 100000.0;
		if (pkt->data[3] == 1) cruise_control_pi.k_d = (float) unpacku32(&pkt->data[12]) / 100000.0;
		B_tcpSend(btcp, pkt->data, pkt->length); // ack
      }
	}
  xTaskResumeAll();
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

//void PSMVoltageTaskHandler(void* parameters){
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


void PSMTaskHandler(void * parameters){
	// we want to read voltage at half the frequency of current to avoid cpu overload
	int read_volt_counter_default_val = 1;
	int read_volt_counter = read_volt_counter_default_val;

	float current;
	float voltage;

	int delay = pdMS_TO_TICKS(round(1000 / PSM_FIR_FILTER_SAMPLING_FREQ_MCMB_CURRENT));

	while (1){

		current = readPSM(&psmPeriph, CURRENT, 3);

		if (read_volt_counter == 0){
			voltage = readPSM(&psmPeriph, VBUS, 3);
			vTaskSuspendAll();
			psmFilter.push(&psmFilter, (float) voltage, VOLTAGE_MEASUREMENT);
			xTaskResumeAll();
			read_volt_counter = read_volt_counter_default_val;
		} else{
			read_volt_counter--;
		}

		vTaskSuspendAll();
		sfq_push(&current_queue, current);
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

	float HV_voltage = psmFilter.get_average(&psmFilter, VOLTAGE_MEASUREMENT);
	float HV_current = sfq_get_avg(&current_queue);
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
