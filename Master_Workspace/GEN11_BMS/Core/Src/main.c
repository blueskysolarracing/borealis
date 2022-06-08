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
#define MY_ID 0 //ID of this BMS (needed to determine if BBMB is talking to me or another BMS)
// ^^^^^^^^^^^^^^ NEED TO UPDATE FOR EVERY BMS BEFORE FLASHING ^^^^^^^^^^^^^^//

#define NUM_CELLS 6
#define NUM_TEMP_SENSE 3
#define SEND_PERIOD 200
#define MEAS_PERIOD 200

#define OV_THRESHOLD 4.2
#define UV_THRESHOLD 2.5
#define OT_THRESHOLD 60

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

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint8_t dataToSend[16];
int messageInBinary; //write this in binary. This goes into [LTC6810CommandGenerate]
//uint8_t dataToReceive[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };//voltage data from LTC6810 via SPI

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void LTC6810_callback(TimerHandle_t xTimer);

void send_temp_volt();
void send_error_msg(uint8_t cell_id, uint8_t error_code,  float data_to_send);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--- COMMS ---///
B_uartHandle_t* buart;
B_tcpHandle_t* btcp;

EKF_Battery battery;

float voltage_array[NUM_CELLS]; //Voltage measurements
float SoC_array[NUM_CELLS]; //State of Charge of each cell
float temperature_array[NUM_TEMP_SENSE]; //Temperature measurements

//--- LTC6810 ---//
uint8_t start_LTC6810_meas = 1; //Flag to start LTC6810 measurements
int temp1, temp2, temp3, temp4, temp5, temp6; // data received via spi, msb which?
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_TIM7_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  //--- MCU OK LED ---//
  NVIC_EnableIRQ(TIM7_IRQn);
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef*) &htim7); //Blink LED to show that CPU is still alive

  //--- COMMS ---//
  buart = B_uartStart(&huart1);
  btcp = B_tcpStart(BMS_ID, &buart, buart, 1, &hcrc);

  //--- LTC6810 ---//
  LTC6810Init(&hspi3, 0, 0, 0, 0, 0, 0, 0, 0);

  //--- FREERTOS ---//
  xTimerStart(xTimerCreate("LTC6810_Timer",  pdMS_TO_TICKS(MEAS_PERIOD), pdTRUE, (void *)0, LTC6810_callback), 0); //Temperature and voltage measurements

  //--- SOC ALGO ---//
  initBatteryAlgo(&battery);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
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
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
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
  htim7.Init.Prescaler = 63999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  HAL_GPIO_WritePin(GPIOC, AFE_WDT_Pin|LTC6810_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BBMB_INT_GPIO_Port, BBMB_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_PWR_Pin|EN_BLN_PWR_Pin|LED1_Pin|RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AFE_WDT_Pin LTC6810_CS_Pin */
  GPIO_InitStruct.Pin = AFE_WDT_Pin|LTC6810_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 PC0 PC1
                           PC2 PC3 PC6 PC7
                           PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_PWR_Pin EN_BLN_PWR_Pin LED1_Pin RS485_EN_Pin */
  GPIO_InitStruct.Pin = EN_PWR_Pin|EN_BLN_PWR_Pin|LED1_Pin|RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB12 PB13
                           PB14 PB15 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void serialParse(B_tcpPacket_t *pkt){
	while(1){
		switch(pkt->senderID){
			  case BBMB_ID: //Parse data from BBMB
				  //--- SoC REQUEST FROM BBMB ---//
				  /* BBMB will receive SoC packet, voltage packet and temperature packet*/
				if((pkt->data[0] == BBMB_SOC_REQUEST_ID) && (pkt->data[1] == MY_ID)){ //BBMB is asking SoC from me
					float current;
					current = arrayToFloat(pkt->data + 2);

					//SoC computation for each 14P group (the assumption is that the voltage measurements are recent enough
					for (int i = 0; i < NUM_CELLS; i++){
						SoC_array[i] = run_EKF(&battery.batteryPack[i], current, voltage_array[i]);
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

					send_temp_volt(); //Also, send temp and volt
				}
				break;
			}
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

void LTC6810_callback(TimerHandle_t xTimer){
	//This callback is called periodically to update voltage and temperature measurements
	uint16_t local_voltage_array[6]; //Voltage of each cell
	uint16_t local_temp_array[3]; //Temperature readings

	if (start_LTC6810_meas){
		//---- LTC6810 MEASUREMENTS ----//
		while(1){
			readVolt(local_voltage_array, &hspi3, &huart3); //Places voltage in temp 1, temp
			readTemp(local_temp_array, 0, 0, 0, 0, 0, &hspi3, &huart3); //Places temperature in temp 1, temp
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

		send_temp_volt(); //Also, send temp and volt

		//Update global variables
		taskENTER_CRITICAL();
		for (int i = 0; i < NUM_CELLS; i++){	voltage_array[i] = local_voltage_array[i];	}
		for (int i = 0; i < 3; i++){			temperature_array[i] = local_temp_array[i];	}
		taskEXIT_CRITICAL();
	} else {
		taskYIELD();
	}
}

void send_error_msg(uint8_t cell_id, uint8_t error_code,  float data_to_send){

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
