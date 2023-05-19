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
#include "btcp.h"
#include "bms.h"
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
 CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
B_uartHandle_t* buart;
B_tcpHandle_t* btcp;
Bms bms;
uint8_t heartbeat[2] = {BMS_HEARTBEAT_ID, 0};

BmsModule bms_module; // for testing
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// void HeartbeatHandler(TimerHandle_t xTimer);
void heartbeat_task(void* argument);
void sender_task(void* argument);
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
  MX_CRC_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  //configASSERT(xTimerStart(xTimerCreate("HeartbeatHandler",  pdMS_TO_TICKS(HEARTBEAT_INTERVAL / 2), pdTRUE, (void *)0, HeartbeatHandler), 0)); //Heartbeat handler
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

#ifdef DEFAULT_TASK
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1000);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
#endif
  /* add threads, ... */

  /* =============== TEST ==================*/

//	bms_module_init(&bms_module, 0, &hspi3, GPIOC, GPIO_PIN_9);
//	TaskHandle_t bms_module_while_loop_test_task_handle;
//	BaseType_t status = xTaskCreate(
//			bms_module_while_loop_test,  //Function that implements the task.
//			"bms_module_while_loop_test",  //Text name for the task.
//			256, 		 //256 words *4(bytes/word) = 1024 bytes allocated for task's stack (note even this size is unnecessary as this thread barely creates any variables)
//			&bms_module,  //Parameter passed into the task.
//			4,  //Priority at which the task is created.
//			&bms_module_while_loop_test_task_handle  //Used to pass out the created task's handle.
//		);
//	configASSERT(status == pdPASS);	// Error checking


  /* =============== TEST ==================*/


  /* ================= Actual code ================== */

  /*
   * Array of chip selects used by each BMS Module object
   * Since each object communicates to its corresponding physical BMS Module's LTC6810 device through a shared LTC6820 isoSPI transceiver, all chip selects are meant for LTC6820 and are identical
   */
  GPIO_TypeDef* spi_cs_ports[] = {[0 ... (BMS_NUM_BMS_MODULES-1)] = GPIOC};
  uint16_t spi_cs_pins[] = {[0 ... (BMS_NUM_BMS_MODULES-1)] = GPIO_PIN_9};

  // Initialize and run bms
  bms_init(&bms, &hspi3, spi_cs_ports, spi_cs_pins);


  buart = B_uartStart(&huart1);
  btcp = B_tcpStart(BMS_ID, &buart, buart, 1, &hcrc);
	TaskHandle_t heartbeat_task_handle;
	BaseType_t status = xTaskCreate(
			heartbeat_task,  //Function that implements the task.
			"heartbeat",  //Text name for the task.
			256, 		 //256 words *4(bytes/word) = 1024 bytes allocated for task's stack (note even this size is unnecessary as this thread barely creates any variables)
			NULL,  //Parameter passed into the task.
			4,  //Priority at which the task is created.
			&heartbeat_task_handle  //Used to pass out the created task's handle.
		);
	configASSERT(status == pdPASS);	// Error checking

	TaskHandle_t sender_task_handle;
	status = xTaskCreate(
			sender_task,  //Function that implements the task.
			"sender",  //Text name for the task.
			1024, 		 //1024 words *4(bytes/word) = 4096 bytes allocated for task's stack
			NULL,  //Parameter passed into the task.
			4,  //Priority at which the task is created.
			&sender_task_handle  //Used to pass out the created task's handle.
		);
	configASSERT(status == pdPASS);	// Error checking
  /* ================= Actual code ================== */


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
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LTC6820_CS_GPIO_Port, LTC6820_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LTC6820_CS_Pin */
  GPIO_InitStruct.Pin = LTC6820_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LTC6820_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DE_Pin */
  GPIO_InitStruct.Pin = RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void serialParse(B_tcpPacket_t *pkt){
	char buf[] = "received from BBMB\n\n";
	switch(pkt->senderID){
	  case BBMB_ID: //Parse data from BBMB

		  		HAL_UART_Transmit(&huart3, (uint8_t*)buf, sizeof(buf), 100);

		if (pkt->data[0] == BBMB_BUS_METRICS_ID) {
			float current = arrayToFloat(&(pkt->data[8])); //Battery current
			if (current > -100 && current < 100) {
				if (bms.init_flag) {
					bms.set_current(&bms, current);
				}
			}
		}
		break;
	}
}

// void HeartbeatHandler(TimerHandle_t xTimer){
// 	//Send periodic heartbeat so we know the board is still running
// 	B_tcpSend(btcp, heartbeat, sizeof(heartbeat));
// 	heartbeat[1] = ~heartbeat[1]; //Toggle for next time
// 	//Heartbeat only accessed here so no need for mutex
// }
void heartbeat_task(void* argument)
{

	while(1) {
		B_tcpSend(btcp, heartbeat, sizeof(heartbeat));
		char buf[] = "heartbeat\n\n";
		HAL_UART_Transmit(&huart3, (uint8_t*)buf, sizeof(buf), 100);
		heartbeat[1] = ~heartbeat[1];
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void sender_task(void* argument) {

	while (!bms.init_flag) {}

	while(1) {
		// Send data to BBMB
		for (int module_id = 0; module_id < BMS_NUM_BMS_MODULES; module_id++) {
			/* ================= SOC =================== */
			float state_of_charges[BMS_MODULE_NUM_STATE_OF_CHARGES];
			if (bms.get_state_of_charge(&bms, state_of_charges, module_id)) {
				// If module x has only 4 cells,  we place a fake SoC of 1.5 (so stategy app can detect it and ignore it) so displays doesn't show the wrong SoC
				//if (module_id == X) state_of_charges[4] = 1.5;

				// Pack state_of_charges into array of uint8_t to be sent to BBMB
				uint8_t buf_soc[4 + BMS_MODULE_NUM_STATE_OF_CHARGES*sizeof(float)];	//[DATA ID, MODULE ID, SoC group #0, SoC group #1, ... , SoC group #4]
				buf_soc[0] = BMS_CELL_SOC_ID;
				buf_soc[1] = module_id;
				for (int i = 0; i < BMS_MODULE_NUM_STATE_OF_CHARGES; i++)
					floatToArray(state_of_charges[i], buf_soc + (i + 1) * sizeof(float));
				B_tcpSend(btcp, buf_soc, sizeof(buf_soc));
			}


			/* =============== Temperature ================= */
			float temperatures[BMS_MODULE_NUM_TEMPERATURES];
			if (bms.get_temperature(&bms, temperatures, module_id, GET_PAST_AVERAGE)) {
				uint8_t buf_temp[4 + BMS_MODULE_NUM_TEMPERATURES*sizeof(float)];
				buf_temp[0] = BMS_CELL_TEMP_ID;
				buf_temp[1] = module_id;
				for (int i = 0; i < BMS_MODULE_NUM_TEMPERATURES; i++)
					floatToArray(temperatures[i], buf_temp + (i + 1) * sizeof(float));
				B_tcpSend(btcp, buf_temp, sizeof(buf_temp));
			}


			/* =============== Voltage ====================== */
			float voltages[BMS_MODULE_NUM_VOLTAGES];
			if (bms.get_voltage(&bms, voltages, module_id, GET_PAST_AVERAGE)) {
				uint8_t buf_volt[4 + BMS_MODULE_NUM_VOLTAGES*sizeof(float)];
				buf_volt[0] = BMS_CELL_VOLT_ID;
				buf_volt[1] = module_id;
				for (int i = 0; i < BMS_MODULE_NUM_VOLTAGES; i++)
					floatToArray(voltages[i], buf_volt + (i + 1) * sizeof(float));
				B_tcpSend(btcp, buf_volt, sizeof(buf_volt));
			}
			vTaskDelay(pdMS_TO_TICKS(200));
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
