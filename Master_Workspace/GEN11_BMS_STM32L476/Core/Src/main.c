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
#include "batteryEKF.h"
#include "LTC6810.h"
#include "timers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// -------------- NEED TO UPDATE FOR EVERY BMS BEFORE FLASHING --------------//
#define MY_ID 1 //ID of this BMS (needed to determine if BBMB is talking to me or another BMS)
// ^^^^^^^^^^^^^^ NEED TO UPDATE FOR EVERY BMS BEFORE FLASHING ^^^^^^^^^^^^^^//

#define NUM_CELLS 5 //have to make sure is the same as NUM_14P_UNITS in batteryEKF.h
#define NUM_TEMP_SENSE 3
#define SEND_PERIOD 200
#define MEAS_PERIOD 200

#define OV_THRESHOLD 4.2
#define UV_THRESHOLD 2.5
#define OT_THRESHOLD 40 //Should be set to 60C

#define TEMP_CORRECTION_MULTIPLIER 1.0
#define TEMP_CORRECTION_OFFSET 0.0

#define TCP_ID BMS_ID
#define LTC6810_INTERVAL 200 //Interval of reading LTC6810 measurements
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
B_uartHandle_t* buart;
B_tcpHandle_t* btcp;

//--- SOC ---//
EKF_Battery inBattery;
long tickLastMeasurement[NUM_CELLS]; //Contains the tick value of the last measurement sent for every cell

uint8_t dataToSend[16]; //data organized by function [LTC6810CommandGenerate]
int messageInBinary; //write this in binary. This goes into [LTC6810CommandGenerate]
uint8_t dataToReceive[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };//voltage data from LTC6810 via SPI
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void LTC6810Handler(TimerHandle_t xTimer);
int LTC6810Init(int GPIO4, int GPIO3, int GPIO2 ,int DCC5, int DCC4, int DCC3, int DCC2, int DCC1);
int readTemp(float*tempArray, int DCC5, int DCC4, int DCC3, int DCC2, int DCC1);//DCC5~1 passed in so no mess up discharge
int readVolt(float*voltArray);
void send_error_msg(uint8_t cell_id, uint8_t error_code,  float data_to_send);
void convert_to_temp(float input_voltage[3], float output_temperature[3]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float voltage_array[NUM_CELLS];
float temperature_array[NUM_TEMP_SENSORS];
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
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_CRC_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //--- COMMS ---//
  buart = B_uartStart(&huart1);
  btcp = B_tcpStart(BMS_ID, &buart, buart, 1, &hcrc);

  //--- MCU OK LED ---//
  NVIC_EnableIRQ(TIM7_IRQn);
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef*) &htim7); //Blink LED to show that CPU is still alive

  //--- FREERTOS ---//
  xTimerStart(xTimerCreate("LTC6810Handler",  pdMS_TO_TICKS(LTC6810_INTERVAL), pdTRUE, (void *)0, LTC6810Handler), 0); //Temperature and voltage measurements

  //--- BMS_FLT ---//
  HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_SET); //Signal is active-high (high when no fault)

  //--- SOC algorithm ---//
  float local_voltage_array[6]; //Initial voltage of each cell
  readVolt(local_voltage_array); //Places voltage in temp 1, temp

  float tickNow = HAL_GetTick();
  for(int i=0; i<NUM_CELLS; i++){
	  tickLastMeasurement[i] = tickNow;
  }
  initBatteryAlgo(&inBattery, &local_voltage_array[0], tickNow);

//--- TESTING ---//
  char buf[5] = "test\n";
  while(1){
	  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 100);
	  HAL_Delay(50);
  }

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim7.Init.Prescaler = 64000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|LTC6810_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_PWR_Pin|EN_BLN_PWR_Pin|MCU_LED_Pin|RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 LTC6810_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|LTC6810_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_WKUP_Pin */
  GPIO_InitStruct.Pin = BMS_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMS_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BBMB_INT_Pin */
  GPIO_InitStruct.Pin = BBMB_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BBMB_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PSENSE_ALERT_Pin */
  GPIO_InitStruct.Pin = PSENSE_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PSENSE_ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_PWR_Pin EN_BLN_PWR_Pin MCU_LED_Pin RS485_EN_Pin */
  GPIO_InitStruct.Pin = EN_PWR_Pin|EN_BLN_PWR_Pin|MCU_LED_Pin|RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void serialParse(B_tcpPacket_t *pkt){
	switch(pkt->senderID){
	  case BBMB_ID: //Parse data from BBMB
		  //--- SoC REQUEST FROM BBMB ---//
		  /* BBMB will receive SoC packet, voltage packet and temperature packet*/
		if((pkt->data[0] == BBMB_STATE_OF_CHARGE_ID) && (pkt->data[1] == MY_ID)){ //BBMB is asking SoC from me
			float current;
			float SoC_array[NUM_CELLS];
			current = arrayToFloat(pkt->data + 2);

			//SoC computation for each 14P group (the assumption is that the voltage measurements are recent enough)
			for (int i = 0; i < NUM_CELLS; i++){
				long currentTick = HAL_GetTick();
				run_EKF(&inBattery.batteryPack[i], currentTick - tickLastMeasurement[i], current, voltage_array[i]);
				SoC_array[i]=inBattery.batteryPack[i].stateX[0];
				tickLastMeasurement[i] = currentTick;
			}

			//Send back SoC for all cells
			uint8_t buf_soc[4 + 5*4]; //[DATA ID, MODULE ID, SoC group #0, SoC group #1, ... , SoC group #4]
			buf_soc[0] = BMS_CELL_SOC_ID;
			buf_soc[1] = MY_ID;

			//Pack SoCs into buffer
			for (int i = 0; i < 5; i++){
				if (i >= NUM_CELLS){
					for (int j = 0; j < sizeof(float); j++){	buf_soc[4 + sizeof(float) * i + j] = -1; }	//Just fill with -1 if it is to be empty

				} else {
					uint8_t floatArray[4];
					floatToArray(SoC_array[i], floatArray); //Convert float into array

					//Pack into buffer
					for (int j = 0; j < sizeof(float); j++){
						buf_soc[4 + sizeof(float) * i + j] = floatArray[j];
					}
				}
			}
			//Send
			HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET); //Enable RS485 driver
			B_tcpSend(btcp, buf_soc, sizeof(buf_soc));
			HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); //Disable RS485 driver

			//Also, send temp and volt
			send_temp_volt();
		}
		break;
	}
}

void send_temp_volt(){
	//--- SEND TEMPERATURE ---//
		uint8_t buf_temp[4 + 6*4]; //[DATA ID, MODULE ID, Temp group #0, Temp group #1, ... , Temp group #4]
		buf_temp[0] = BMS_CELL_TEMP;
		buf_temp[1] = MY_ID;

		//Pack temperatures into buffer
		for (int i = 0; i < 6; i++){
			if (i >= 3){ //Thermistors not populated
				for (int j = 0; j < sizeof(float); j++){	buf_temp[4 + sizeof(float) * i + j] = -1; }	//Just fill with -1 if it is to be empty

			} else {
				uint8_t floatArray[4];
				floatToArray(temperature_array[i], floatArray); //Convert float into array

				//Pack into buffer
				for (int j = 0; j < sizeof(float); j++){
					buf_temp[4 + sizeof(float) * i + j] = floatArray[j];
				}
			}
		}

		//Send
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET); //Enable RS485 driver
		B_tcpSend(btcp, buf_temp, sizeof(buf_temp));
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); //Disable RS485 driver

	//--- SEND VOLTAGE ---//
		uint8_t buf_voltage[4 + 6*4]; //[DATA ID, MODULE ID, Voltage group #0, Voltage group #1, ... , Voltage group #4]
		buf_voltage[0] = BMS_CELL_VOLT;
		buf_voltage[1] = MY_ID;

		//Pack voltage into buffer
		for (int i = 0; i < 6; i++){
			if (i >= NUM_CELLS){ //Thermistors not populated
				for (int j = 0; j < sizeof(float); j++){	buf_voltage[4 + sizeof(float) * i + j] = -1; }	//Just fill with -1 if it is to be empty

			} else {
				uint8_t floatArray[4];
				floatToArray(voltage_array[i], floatArray); //Convert float into array

				//Pack into buffer
				for (int j = 0; j < sizeof(float); j++){
					buf_voltage[4 + sizeof(float) * i + j] = floatArray[j];
				}
			}

		//Send
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET); //Enable RS485 driver
		B_tcpSend(btcp, buf_voltage, sizeof(buf_voltage));
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); //Disable RS485 driver
	}
}

void LTC6810Handler(TimerHandle_t xTimer){
	//This callback is called periodically to update voltage and temperature measurements
	float local_voltage_array[6]; //Voltage of each cell
	float local_temp_array[3]; //Temperature readings

	//---- LTC6810 MEASUREMENTS ----//
	readVolt(local_voltage_array); //Places voltage in temp 1, temp
	readTemp(local_temp_array, 0, 0, 0, 0, 0); //Places temperature in temp 1, temp
	convert_to_temp(local_temp_array, local_temp_array);

	while (1){
		uint8_t junk[3] = {3,2,5};
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET); //Enable RS485 driver
		B_tcpSend(btcp, junk, sizeof(junk));
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); //Disable RS485 driver
		HAL_Delay(100);
	}

	//---- BATTERY FAULT CHECK----//
	//Check for UV, OV faults
	for (int i = 0; i < 5; i++){
		if (local_voltage_array[i] > OV_THRESHOLD){ //Overvoltage
			HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_RESET); //OV protection tripped, put car into safe state
			htim7.Instance->PSC = 10667; //Flash LED 6 times per sec (assuming 64MHz clock)
			HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_RESET); //Switch to 12V power
			send_error_msg(i, BMS_OV, voltage_array[i]);

		} else if (local_voltage_array[i] < UV_THRESHOLD){ //Undervoltage
			HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_RESET); //UV protection tripped, put car into safe state
			htim7.Instance->PSC = 10667; //Flash LED 6 times per sec (assuming 64MHz clock)
			send_error_msg(i, BMS_UV, voltage_array[i]);

		} else { //Voltage OK
			HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_SET); //Battery voltages OK
		}
	}

	//Check for OT faults
	for (int i = 0; i < 6; i++){
		if (local_temp_array[i] > OT_THRESHOLD){
			HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_RESET); //OT protection tripped, put car into safe state
			htim7.Instance->PSC = 10667; //Flash LED 6 times per sec (assuming 64MHz clock)
			send_error_msg(i, BMS_OT, temperature_array[i]);

		} else {
			HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_SET); //Battery voltages OK
		}
	}

	//Update global variables
	taskENTER_CRITICAL();
	for (int i = 0; i < NUM_CELLS; i++){	voltage_array[i] = local_voltage_array[i];	}
	for (int i = 0; i < 3; i++){			temperature_array[i] = local_temp_array[i];	}
	taskEXIT_CRITICAL();
	send_temp_volt();
	osDelay(SEND_PERIOD);
}

void send_error_msg(uint8_t cell_id, uint8_t error_code,  float data_to_send){

}

int LTC6810Init(int GPIO4, int GPIO3, int GPIO2 ,int DCC5, int DCC4, int DCC3, int DCC2, int DCC1) {
	//This function initialize the LTC6810 ADC chip, shall be called at the start of the program
	//To modify the initialization setting, please refer to the Datasheet and the chart below
	/*
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | reg   | Bit7    | Bit6    | Bit5    | Bit4    | Bit3    | Bit2    | Bit1    | Bit0   |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR0 | RSVD    | GPIO4   | GPIO3   | GPIO2   | GPIO1   | REFON   | DTEN    | ADCOPT |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR1 | VUV[7]  | VUV[6]  | VUV[5]  | VUV[4]  | VUV[3]  | VUV[2]  | VUV[1]  | VUV[0] |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR2 | VOV[3]  | VOV[2]  | VOV[1]  | VOV[0]  | VUV[11] | VUV[10] | VUV[9]  | VUV[8] |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR3 | VOV[11] | VOV[10] | VOV[9]  | VOV[8]  | VOV[7]  | VOV[6]  | VOV[5]  | VOV[4] |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR4 | DCC0    | MCAL    | DCC6    | DCC5    | DCC4    | DCC3    | DCC2    | DCC1   |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR5 | DCTO[3] | DCTO[2] | DCTO[1] | DCTO[0] | SCONV   | FDRF    | DIS_RED | DTMEN  |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+*/

	//connection to MUX for thermal measurements:
	//GPIO2 -> A0
	//GPIO3 -> A1
	//GPIO4 -> A2
	//GPIO1 shall always be set to 1 to avoid internal pull-down, as its the voltage reading pin.
	//A2 is MSB and A0 is MSB, if want channel 2 -> do A2=0, A1=1, A0=0
	//above is the table of configuration register bits. How
	int messageInBinary;
	int i;
	int data6Byte[48]; //[47] is MSB, CFGR0 Bit7
	uint8_t dataToSend[12]; //stores the command and PEC bit of WRCFG
	//2byte command, 2byte PEC, 6byte data, 2byte PEC

	uint8_t dataToReceive[8];

	//write configure command
	messageInBinary = 0b1;		//write config
	LTC6810CommandGenerate(messageInBinary, dataToSend);
	//set GPIO bits to 1 so they aren`t being pulled down internally by the chip.
	//set REFON to enable the 3V that goes to the chip
	//DTEN to 0 to disable discharge timer
	//ADCOPT bit to 0, use 422Hz as its stable
	//above are byte0, byte 1 full of 0s as VUV currently not used.
	messageInBinary = 0b0000110000000000; //CFGR0&1
	//now add the GPIO config
	messageInBinary = messageInBinary + 4096 * GPIO2 + 8192 * GPIO3
			+ 16384 * GPIO4;

	int MSG0[8];
	int MSG1[8];
	commandToArray(messageInBinary, MSG0, MSG1);
	for (i = 7; i >= 0; i--) {
		data6Byte[40 + i] = MSG0[i];
		data6Byte[32 + i] = MSG1[i];
	}

	//set the VOV and VUV as 0;
	messageInBinary = 0b0000000000000000; //CFGR2&3
	int MSG2[8];
	int MSG3[8];
	commandToArray(messageInBinary, MSG2, MSG3);
	for (i = 7; i >= 0; i--) {
		data6Byte[24 + i] = MSG2[i];
		data6Byte[16 + i] = MSG3[i];
	}

	//DCC = 0: discharge off, 1: discharge = on
	//MCAL = 1: enable multi-calibration. for this don`t use, turn to 0
	//DCTO: discharge timer. for now 0
	//SCONV: redundant measurement using S pin, disable for now (0)
	//FDRF: not using it anyway, to 0
	//DIS_RED: redundancy disable: set to 1 to disable
	//DTMEN: Discharge timer monitor, 0 to disable
	messageInBinary = 0b0000000000000010;
	messageInBinary += 256*DCC1 + 512*DCC2 + 1024*DCC3 + 2048*DCC4 +4096*DCC5;

	int MSG4[8];
	int MSG5[8];
	commandToArray(messageInBinary, MSG4, MSG5);
	for (i = 7; i >= 0; i--) {
		data6Byte[8 + i] = MSG4[i];
		data6Byte[0 + i] = MSG5[i];
	}

	//now create PEC bits based on above data
	int PEC0_6[8];
	int PEC1_6[8];
	generatePECbits6Byte(data6Byte, PEC0_6, PEC1_6);

	//change array back to bytes
	dataToSend[4] = arrayToByte(MSG0);
	dataToSend[5] = arrayToByte(MSG1);
	dataToSend[6] = arrayToByte(MSG2);
	dataToSend[7] = arrayToByte(MSG3);
	dataToSend[8] = arrayToByte(MSG4);
	dataToSend[9] = arrayToByte(MSG5);
	dataToSend[10] = arrayToByte(PEC0_6);
	dataToSend[11] = arrayToByte(PEC1_6);

	//now send the data
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //slave select low, transmit begin
	HAL_Delay(1); //require several us
	HAL_SPI_Transmit(&hspi3, dataToSend, 12, 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

	return 0; //temp
}

int readTemp(float*tempArray, int DCC5, int DCC4, int DCC3, int DCC2, int DCC1){
	//read Temp 0,1,2. Pass by reference to the input array.
	//if discharging, make DCC global variables so this don't disturb Discharge

	int cycle = 0;
while(cycle<3){

	if(cycle ==0){
	LTC6810Init(0,0,0,DCC5, DCC4, DCC3, DCC2, DCC1);}//channel 1
	else if(cycle == 1){
	LTC6810Init(0,0,1,DCC5, DCC4, DCC3, DCC2, DCC1);}//channel 2
	else if(cycle == 2){
	LTC6810Init(0,1,0,DCC5, DCC4, DCC3, DCC2, DCC1);}//channel 3
//cycle are 0,1,2

	messageInBinary = 0b10100010010;  //conversion GPIO1, command AXOW
	LTC6810CommandGenerate(messageInBinary, dataToSend);
	HAL_Delay(1);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);//slave low
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi3, dataToSend, 4/*byte*/, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

	HAL_Delay(2);
	//now read from it
	messageInBinary = 0b1100; //read auxiliary group 1, command RDAUXA
			  //now receive those data
	LTC6810CommandGenerate(messageInBinary, dataToSend);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi3, dataToSend, 4/*byte*/, 100);
	HAL_Delay(2); //ADD DELAY between Transmit & Receive
	HAL_SPI_Receive(&hspi3, dataToReceive, 6, 100);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	//write value into array
	int tempSum = 256*dataToReceive[3] + dataToReceive[2];
	//float TempInC = 1/(1/298.15+(1/3950) * log((-10)/(tempSum/)))
	tempArray[cycle] = (float)tempSum;//msb of data
	cycle++;
	}//end of while(cycle = sth)
return 0;

}

int readVolt(float*voltArray){
	int VmessageInBinary;//for internal command

	  //first read first half of data
	  VmessageInBinary = 0b01101110000; //adcv discharge enable,7Hz
	  LTC6810CommandGenerate(VmessageInBinary, dataToSend);//generate the "check voltage command"
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //slave select low, transmit begin
	  HAL_Delay(1); //require several us
	  HAL_SPI_Transmit(&hspi3, dataToSend, 4/*byte*/, 100);
	  //now receive those data
	  HAL_Delay(4); //ADD DELAY between Transmit & Receive
	  HAL_SPI_Receive(&hspi3, dataToReceive, 8, 100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

	  //Maybe Try fix this BUG???????

	  VmessageInBinary = 0b100;  //read cell voltage reg group 1;
	  LTC6810CommandGenerate(VmessageInBinary, dataToSend);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //slave select low, transmit begin
	  HAL_Delay(1); //require several us
	  HAL_SPI_Transmit(&hspi3, dataToSend, 4/*byte*/, 100);
	  HAL_Delay(3); //ADD DELAY between Transmit & Receive
	  HAL_SPI_Receive(&hspi3, dataToReceive, 8, 100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);


	  voltArray[0] = voltageDataConversion(dataToReceive[0], dataToReceive[1]) /10000.0;
	  voltArray[1] = voltageDataConversion(dataToReceive[2], dataToReceive[3]) /10000.0;
	  voltArray[2] = voltageDataConversion(dataToReceive[4], dataToReceive[5]) /10000.0;

	  /*
	  v1 = voltageDataConversion(dataToReceive[0], dataToReceive[1]) ;
	  v2 = voltageDataConversion(dataToReceive[2], dataToReceive[3]) ;
	  v3 = voltageDataConversion(dataToReceive[4], dataToReceive[5]);
	   */
	  VmessageInBinary = 0b110; //read cell voltage reg group 2;
	  LTC6810CommandGenerate(VmessageInBinary, dataToSend);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //slave select low, transmit begin
	  HAL_Delay(1); //require several us
	  HAL_SPI_Transmit(&hspi3, dataToSend, 4/*byte*/, 100);
	  HAL_Delay(3); //ADD DELAY between Transmit & Receive
	  HAL_SPI_Receive(&hspi3, dataToReceive, 8, 100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

	  voltArray[3] = voltageDataConversion(dataToReceive[0], dataToReceive[1]) /10000.0;
	  voltArray[4] = voltageDataConversion(dataToReceive[2], dataToReceive[3]) /10000.0;
	  voltArray[5] = voltageDataConversion(dataToReceive[4], dataToReceive[5]) /10000.0;
	  /*
	  v4 = voltageDataConversion(dataToReceive[0], dataToReceive[1]) ;
	  v5 = voltageDataConversion(dataToReceive[2], dataToReceive[3]) ;
	  v6 = voltageDataConversion(dataToReceive[4], dataToReceive[5]) ;
	   */
}

void convert_to_temp(float input_voltage[3], float output_temperature[3]){
	/* Used to convert raw ADC code to temperature */
	for (int i = 0; i < 3; i++){
		float corrected_voltage = TEMP_CORRECTION_MULTIPLIER * (input_voltage[i] / 10000.0  - TEMP_CORRECTION_OFFSET);
		float thermistor_resistance = 10.0 / ((2.8 / (float) corrected_voltage) - 1.0);
		output_temperature[i] = 1.0 / (0.003356 + 0.0002532 * log(thermistor_resistance / 10.0));
		output_temperature[i] = output_temperature[i] - 273.15;
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
  else if (htim->Instance == TIM7){
	HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
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
