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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "hrtim.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buart.h"
#include "btcp.h"
#include "protocol_ids.h"
//#include "bmisc.h"
//#include "psm.h"
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//--- COMMS ---//
B_tcpHandle_t btcp_bms_actual;
B_tcpHandle_t btcp_main_actual;

B_uartHandle_t* buart_main;
B_uartHandle_t* buart_bms;
B_tcpHandle_t* btcp_main = &btcp_main_actual;
B_tcpHandle_t* btcp_bms = &btcp_bms_actual;
uint8_t heartbeat[2] = {BBMB_HEARTBEAT_ID, 0};

//--- BMS ---//
uint8_t BMS_requesting_from = 7; //Holds which BMS we are requesting data from (needs initial value > 6)
uint8_t BMS_data_received[3] = {1, 1, 1}; //Holds whether we received voltage, temperature and SoC

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_UART8_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_UART7_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  //--- MCU OK LED ---//
  NVIC_EnableIRQ(TIM7_IRQn);
  HAL_TIM_Base_Start_IT((TIM_HandleTypeDef*) &htim7); //Blink LED to show that CPU is still alive

  //--- COMMS ---//
  buart_main = B_uartStart(&huart4);
  btcp_main = B_tcpStart(BBMB_ID, &buart_main, buart_main, 1, &hcrc);
  buart_bms = B_uartStart(&huart8);
  btcp_bms = B_tcpStart(BBMB_ID, &buart_bms, buart_bms, 1, &hcrc);

  hspi2;
  return;

  //--- FREERTOS ---//
//  configASSERT(xTimerStart(xTimerCreate("HeartbeatHandler",  pdMS_TO_TICKS(HEARTBEAT_INTERVAL / 2), pdTRUE, (void *)0, HeartbeatHandler), 0)); //Heartbeat handler
//  configASSERT(xTimerStart(xTimerCreate("BMSPeriodicReadHandler",  pdMS_TO_TICKS(BMS_READ_INTERVAL), pdTRUE, (void *)0, BMSPeriodicReadHandler), 0)); //Read from BMS periodically

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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

/* USER CODE BEGIN 4 */
void BMSPeriodicReadHandler(TimerHandle_t xTimer){
//	/*Used to kickoff a new cycle of BMS data requests.
//	* Request data from first BMS, and upon reception, serialParse will request from others
//	*/
//
//	//Sending junk just to test
//	char junk[2] = {3, 1};
//	B_tcpSend(btcp_bms, junk, sizeof(junk));
//	B_tcpSend(btcp_main, junk, sizeof(junk));
//
//
//	taskENTER_CRITICAL();
//	if (BMS_requesting_from > 6){ //We've received from all BMS, start requesting from the first one again
//
//		BMS_requesting_from = 1;
//
//		uint8_t BMS_Request[2 * 4] = {0};
//		BMS_Request[0] = BBMB_BMS_DATA_REQUEST_ID;
//		BMS_Request[1] = BMS_requesting_from;
//		floatToArray((float) battery_current, BMS_Request + 4); //Send current
//		B_tcpSend(btcp_bms, BMS_Request, sizeof(BMS_Request));
//
//	}
//	taskEXIT_CRITICAL();
}

void serialParse_BBMB(B_tcpPacket_t *pkt){

	uint8_t buf[100];

	switch(pkt->senderID){
		case DCMB_ID: //Parse data from DCMB
			if (pkt->payload[0] == DCMB_LIGHTCONTROL_ID){
				xQueueSend(lightsCtrl, &(pkt->payload[1]), 200); //Send to lights control task
			} //else if (pkt->payload[0] == DCMB_CAR_STATE_ID){
//					uint8_t relay_open_cmd;
//					if ((pkt->payload[1] == CAR_SAFE_STATE) || (pkt->payload[1] == CAR_SLEEP)){ //Need to open power relays
//						relay_open_cmd = 1;
//						xQueueSend(relayCtrl, &relay_open_cmd, 200); //Open relays (next time relayTask runs)
//						HAL_GPIO_WritePin(BMS_NO_FLT_GPIO_Port, BMS_NO_FLT_Pin, GPIO_PIN_RESET); //Switch to supplemental supply
//
//					} else if (pkt->payload[1] == CAR_CHARGING_SOLAR){ //Need to close power relays
//						relay_open_cmd = 2;
//						xQueueSend(relayCtrl, &relay_open_cmd, 200); //Close relays (next time relayTask runs)
//						HAL_GPIO_WritePin(BMS_NO_FLT_GPIO_Port, BMS_NO_FLT_Pin, GPIO_PIN_RESET); //Switch to supplemental supply
//
//					} else if (pkt->payload[1] == CAR_DRIVE){ //Need to close power relays
//						relay_open_cmd = 2;
//						xQueueSend(relayCtrl, &relay_open_cmd, 200); //Close relays (next time relayTask runs)
//						HAL_GPIO_WritePin(BMS_NO_FLT_GPIO_Port, BMS_NO_FLT_Pin, GPIO_PIN_SET); //Switch to Vicor 12V
//					}
			break;

		case BMS_ID: //Parse data from BMS (comes from btcp_bms)
			if (pkt->payload[0] == BMS_ERROR_STATUS){ //Received error from BMS
				uint8_t BMS_error[2 * 4] = {0};
				BMS_error[0] = BBMB_RELAY_STATE_ID;
				BMS_error[1] = OPEN;
				if (pkt->payload[1] == BMS_OV){	BMS_error[2] = 0x00;	}
				else if (pkt->payload[1] == BMS_UV){	BMS_error[2] = 0x01;	}
				else if (pkt->payload[1] == BMS_OT){	BMS_error[2] = 0x03;	}

				B_tcpSend(btcp_main, BMS_error, sizeof(BMS_error));

			} else { //Received SoCs, temps or voltage from BMS
				//Simply re-send on main bus
				B_tcpSend(btcp_main, pkt->payload, sizeof(pkt->payload));

				taskENTER_CRITICAL();
				if (BMS_requesting_from <= 6){
					//Update flags to indicate what data we've received
					if (pkt->payload[0] == BMS_CELL_VOLT){
						BMS_data_received[0] = 1;
					} else if (pkt->payload[1] == BMS_CELL_TEMP){
						BMS_data_received[1] = 1;
					} else if (pkt->payload[0] == BMS_CELL_SOC_ID){
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

						B_tcpSend(btcp_bms, BMS_Request, sizeof(BMS_Request));
					}
				taskEXIT_CRITICAL();
				}
			}
			break;
		case CHASE_ID:
			if (pkt->payload[0] == CHASE_LIGHTCONTROL_ID){
				xQueueSend(lightsCtrl, &(pkt->payload[1]), 200); //Send to lights control task
			}
			break;

		case MCMB_ID:
			/*BBMB no need for MCMB data*/


			break;
	}
}
/* USER CODE END 4 */

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
