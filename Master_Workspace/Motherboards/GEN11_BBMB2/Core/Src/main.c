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
uint8_t BMS_data_received[3] = {1, 1, 1}; //Holds whether we received voltage, temperature and SoC

//--- BATTERY ---//
uint8_t batteryState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
static void lightsTask(void * argument);
static void relayTask(void * argument);
void PSMTaskHandler(TimerHandle_t xTimer);
void HeartbeatHandler(TimerHandle_t xTimer);
void BMSPeriodicReadHandler(TimerHandle_t xTimer);
void BMS_Flt_Check(TimerHandle_t xTimer);

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
  PSM_Init(&psmPeriph, 1); //2nd argument is PSM ID
  if (configPSM(&psmPeriph, &hspi2, &huart2, "12", 2000) == -1){ //2000ms timeout
	  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); //Turn on red LED as a warning
  }

  //TODO Add check that battery is healthy
  batteryState = HEALTHY;

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
			} else (pkt->payload[0] == DCMB_RELAYS_STATE_ID){
				taskENTER_CRITICAL();

				if (pkt->payload[2] == OPEN){ //Open relays and resend
					open_relays(&relay);

					uint8_t buf[4] = {BBMB_RELAYS_STATE, batteryState, OPEN, pkt->payload[3]};
					B_tcpSend(btcp_main, BMS_error, sizeof(BMS_error));

				} else if ((pkt->payload[2] == CLOSED) && (batteryState == HEALTHY)){ //Try to open relays
					close_relays(&relay);

				}

				taskEXIT_CRITICAL();
			}
			break;

		case BMS_ID: //Parse data from BMS (comes from btcp_bms)
			if ((pkt->payload[0] == BMS_CELL_TEMP) || (pkt->payload[0] == BMS_CELL_VOLT) || (pkt->payload[0] == BMS_CELL_SOC_ID)){ //Received SoCs, temps or voltage from BMS
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
		taskENTER_CRITICAL();
		batteryState = FAULTED;
		taskEXIT_CRITICAL();

		uint8_t car_state_error[1 * 4];
		car_state_error[0] = BBMB_RELAYS_STATE_ID;
		car_state_error[1] = FAULTED;
		car_state_error[2] = OPEN;

		B_tcpSend(btcp_main, car_state_error, sizeof(car_state_error));

		if (PROTECTION_ENABLE){	open_relays(&relay);	}
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

void HeartbeatHandler(TimerHandle_t xTimer){
	//Send periodic heartbeat so we know the board is still running
	B_tcpSend(btcp_main, heartbeat, sizeof(heartbeat));
	heartbeat[1] = ~heartbeat[1]; //Toggle for next time
	//Heartbeat only accessed here so no need for mutex
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
