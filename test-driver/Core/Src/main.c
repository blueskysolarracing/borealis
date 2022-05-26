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
#include "glcd.h"
#include "fonts/font5x7.h"
#include "fonts/Liberation_Sans20x28_Numbers.h"
#include "fonts/Liberation_Sans17x17_Alpha.h"
#include <time.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SPI_TRANSMIT_TIMEOUT 50 //in ms, arbitrarily chosen
#define MAX_UART_TRANSMIT_TIMEOUT 50 //in ms, arbitrarily chosen

// pins
#define SCK_PORT GPIOB
#define SCK_PIN GPIO_PIN_10
#define SI_PORT GPIOC
#define SI_PIN GPIO_PIN_1
#define CS1_PORT GPIOB
#define CS1_PIN GPIO_PIN_12
#define CS2_PORT GPIOE
#define CS2_PIN GPIO_PIN_3
#define RST1_PORT GPIOD
#define RST1_PIN GPIO_PIN_6
#define RST2_PORT GPIOG
#define RST2_PIN GPIO_PIN_9
#define A0_PORT GPIOD
#define A0_PIN GPIO_PIN_4
#define LEDON_PORT GPIOD
#define LEDON_PIN GPIO_PIN_5

//DEBUG PURPOSES
//#define DELAY_BETWEEN_WRITES
#define DELAY_DURATION_BETWEEN_WRITES 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t correct_Y(uint8_t y){
	return ((y+48)%64);
}

void drawP1Default(int value[4]){
	char* labelsP1[] = {"Solar:", "Motor:", "Battery:"};
	int labelsP1L = 3;
	char* labelspeed = "km/h";

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 5;

	// draw the labels
	for(int i = 0; i < labelsP1L; i++){
		char* label = labelsP1[i];
		int j = 0;
		// char by char cuz draw xy only with char
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
			j++;
		}
		glcd_tiny_draw_char_xy(72, correct_Y(y), 'w');
		// go next rows, these value are just what I think will look good
		y+=23;
	}

	// draw divider line
	glcd_draw_line(81, 0,  81, 63, BLACK);
	int x = 0;
	// draw km/h
	while(labelspeed[x] != 0){
		glcd_tiny_draw_char_xy(94+(x*6), correct_Y(52), labelspeed[x]);
		x++;
	}


	// draw the numbers
	char valueS[4][4];

	glcd_tiny_set_font(Font5x7,5,7,32,127);

	// get it in strings
	for(int i = 0; i < 4; i++){
		// sign
		int v = value[i];
		if(v<0){
			valueS[i][0] = '-';
			v *= -1;
		}
		else{
			valueS[i][0] = '+';
		}
		// hundred
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// tenth
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = ' ';
		}
		// ones
		valueS[i][3] = '0' + v%10;
	}

	// write the 3 small values
	int y = 5;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			glcd_tiny_draw_char_xy(48+(j*6), correct_Y(y), valueS[i][j]);
		}
		y+=23;
	}

	// now write the big speed
	glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
	glcd_draw_char_xy(85, 0, valueS[3][2]);
	glcd_draw_char_xy(105, 0, valueS[3][3]);

	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}

void drawP1Detailed(int value[9]){
	char* labelsP1[] = {"Solar:", "Motor:", "Battery:"};
	int labelsP1L = 3;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 5;

	// draw the labels
	for(int i = 0; i < labelsP1L; i++){
		char* label = labelsP1[i];
		int j = 0;
		// char by char cuz draw xy only with char
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
			j++;
		}
		glcd_tiny_draw_char_xy(72, correct_Y(y), 'w');
		// go next rows, these value are just what I think will look good
		y+=23;
	}

	// draw the numbers
	char valueS[9][4];

	glcd_tiny_set_font(Font5x7,5,7,32,127);

	// get it in strings
	for(int i = 0; i < 9; i++){
		// sign
		int v = value[i];
		if(v<0){
			valueS[i][0] = '-';
			v *= -1;
		}
		else{
			valueS[i][0] = '+';
		}
		// hundred
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// tenth
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = ' ';
		}
		// ones
		valueS[i][3] = '0' + v%10;
	}

	// write the 3 small values
	int y = 5;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			glcd_tiny_draw_char_xy(48+(j*6), correct_Y(y), valueS[i][j]);
		}
		y+=23;
	}

	// write the 6 small values
	int y = 5;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			glcd_tiny_draw_char_xy(48+(j*6), correct_Y(y), valueS[i][j]);
		}
		y+=23;
	}



	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}

// draw p1 labels
void drawP1(){
	drawP1Default();
}

// draw p2 labels
void drawP2(){
	char* labelsP2[] = {"Cruise:", "Light:"};
	int labelsP2L = 2;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	int y = 5;
	for(int i = 0; i < labelsP2L; i++){
		char* label = labelsP2[i];
		int j = 0;
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(50+j*6, correct_Y(y), label[j]);
			j++;
		}
		y+=23;
	}

	glcd_draw_line(44, 0,  44, 63, BLACK);
	glcd_tiny_draw_char_xy(19, correct_Y(52), '%');

	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
}

// pass in array of values in order of cruise, light, regen, battery
void updateP2(int value[4]){
	glcd_tiny_set_font(Font5x7,5,7,32,127);

	// write the 2 on off
	int y = 5;
	for(int i = 0; i < 2; i++){
		char* status = "OFF";
		if(value[i]) status = " ON";
		for(int j = 0; j < 3; j++){
			glcd_tiny_draw_char_xy(95+(j*6), correct_Y(y), status[j]);
		}
		y+=23;
	}

	char* regen = "     ";
	if(value[2]) regen = "REGEN";

	glcd_set_font(Liberation_Sans17x17_Alpha, 17, 17, 'A', 'Z');
	for(int i = 0; i < 5; i++){
		glcd_draw_char_xy(49+i*15, 30, regen[i]);
	}

	// now write the big battery
	glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
	glcd_draw_char_xy(0, 0, '0' + value[3]/10);
	glcd_draw_char_xy(21, 0, '0' + value[3]%10);

	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
}
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
  MX_SPI2_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 x*/
	HAL_GPIO_WritePin(GPIOE, ledtest_Pin, GPIO_PIN_SET);
	HAL_Delay(250);

	HAL_GPIO_WritePin(GPIOE, ledtest_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS2_Pin|ledtest_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, A0_Pin|LEDON_Pin|reset1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(reset2_GPIO_Port, reset2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS2_Pin ledtest_Pin */
  GPIO_InitStruct.Pin = CS2_Pin|ledtest_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin LEDON_Pin reset1_Pin */
  GPIO_InitStruct.Pin = A0_Pin|LEDON_Pin|reset1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : reset2_Pin */
  GPIO_InitStruct.Pin = reset2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(reset2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	glcd_init();
	//glcd_test_hello_world();
	//glcd_test_rectangles();
	// my stuff
	glcd_clear();
	srand(time(NULL));   // Initialization, should only be called once.

	int t = 100;
	int p1values[4] = {420, 805, -404, 69};
	int p2values[4] = {1, 1, 1, 87};

	drawP2();
	updateP2(p2values);

	drawP1();
	updateP1(p1values);
	delay_ms(t);
//	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
//	glcd_test_hello_world();
//	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
//
//
	HAL_GPIO_WritePin(ledtest_GPIO_Port,ledtest_Pin, GPIO_PIN_SET);
	delay_ms(t);
	while(1){
		HAL_GPIO_WritePin(ledtest_GPIO_Port,ledtest_Pin, GPIO_PIN_RESET);
		for(int i = 0; i < 3; i++){
			int r = rand()%5;
			if(rand()%2 == 0){
				r*= -1;
			}
			p1values[i] += r;
		}
		int r = rand()%2;
		if(rand()%2 == 0){
			r*= -1;
		}
		p1values[3] += r;
		drawP1();
		updateP1(p1values);
		delay_ms(t);

		for(int i = 0; i < 3; i++){
			int s = rand()%2;
			p2values[i] = s;
		}
		int s = rand()%2;
		if(rand()%2 == 0){
			s*= -1;
		}
		p2values[3] += s;
		drawP2();
		updateP2(p2values);
		delay_ms(t);
		HAL_GPIO_WritePin(ledtest_GPIO_Port,ledtest_Pin, GPIO_PIN_SET);
		delay_ms(t);
	}

	// my stuff


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

