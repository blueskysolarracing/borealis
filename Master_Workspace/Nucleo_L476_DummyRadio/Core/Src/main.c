/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "pack_data.h"
#include "protocol_ids.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BSSR_SERIAL_START 0xa5
#define BSSR_SERIAL_ESCAPE 0x5a
#define BBMB_ADDRESS 0x01
#define PPTMB_ADDRESS 0x02
#define MCMB_ADDRESS 0x03
#define DCMB_ADDRESS 0x04

#define MAX_PACKET_SIZE 256
#define HEADER_LENGTH 4
#define CRC_LENGTH 4

//Data ID 00
#define BUS_METRICS_LENGTH 12
#define MC2_STATE_LENGTH 8
//Data ID 01
#define CELL_METRICS_LENGTH 20 //20 for each cell and there are 29 cell in total, single cell data send at once
#define PPT_METRICS_LENGTH 28 //16 for each ppt and there are 3 ptt in total + 4 non repeating initial bytes
#define SPEED_PULSE_READING_LENGTH 4
#define BBOX_STARTUP_LENGTH 4
//Data ID 02
#define BSD_LENGTH 4
#define MOTOR_TEMPERATURE_LENGTH 4
#define PPTBOX_STARTUP_LENGTH 4
//Data ID 03
#define BMS_MCU_STATUS_LENGTH 16
#define LIGHT_STATE_LENGTH 4
#define SUPP_BATTERY_LENGTH 8
//Data ID 04
#define STEERING_WHEEL_LENGTH 4
//Data ID 05
#define BMS_DATA_REQUEST_LENGTH 8
#define MOTOR_CONTROL_STATE_LENGTH 12
//Data ID 06
#define RELAY_STATE_LENGTH 8
//Data ID 07
#define PEDAL_ANGLE_LENGTH 12
#define BMS_CELL_TEMP_LENGTH 24
//Data ID 08
#define SIDE_PANEL_LENGTH 4
#define BMS_CELL_VOLT_LENGTH 24
//Data ID 09
#define BMS_CELL_SOC_LENGTH 24

//Data ID 0B
#define TEXT_STRING_LENGTH 4
//Data ID 0D
#define LP_BUS_METRICS_LENGTH 20
//Data ID 0E
#define CORE_TEMP_LENGTH 4
//Data ID 0F
#define  HEARTBEAT_LENGTH 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

uint8_t payload[1000];

//Data ID 00
uint8_t Bus_Metrics_Payload[BUS_METRICS_LENGTH];
uint8_t MC2_State_Payload[MC2_STATE_LENGTH];
//Data ID 01
uint8_t Cell_Metrics_Payload[CELL_METRICS_LENGTH];
uint8_t PPT_Metrics_Payload[PPT_METRICS_LENGTH];
uint8_t Speed_Pulse_Reading_Payload[SPEED_PULSE_READING_LENGTH];
uint8_t BBox_Startup_Payload[BBOX_STARTUP_LENGTH];
//Data ID 02
uint8_t BSD_Payload[BSD_LENGTH];
uint8_t Motor_Temperature_Payload[MOTOR_TEMPERATURE_LENGTH];
uint8_t PPTBox_Startup_Payload[PPTBOX_STARTUP_LENGTH];
//Data ID 03
uint8_t BMS_MCU_Status_Payload[BMS_MCU_STATUS_LENGTH];
uint8_t Light_State_Payload[LIGHT_STATE_LENGTH];
uint8_t Supp_Battery_Metric_Payload[SUPP_BATTERY_LENGTH];
//Data ID 04
uint8_t Streeting_Wheel_Payload[STEERING_WHEEL_LENGTH];
//Data ID 05
uint8_t BMS_Data_request[BMS_DATA_REQUEST_LENGTH];
uint8_t Motor_Control_State_Payload[MOTOR_CONTROL_STATE_LENGTH];
//Data ID 06
uint8_t Relay_State[RELAY_STATE_LENGTH];
//Data ID 07
uint8_t Pedal_Angle[PEDAL_ANGLE_LENGTH];
//Data ID 08
uint8_t Side_Panel[SIDE_PANEL_LENGTH];
//Data ID 0B
uint8_t Text_String[TEXT_STRING_LENGTH];
//Data ID 0D
uint8_t LP_Bus_Metrics_Payload[LP_BUS_METRICS_LENGTH];
//Data ID 0E
uint8_t Core_Temp_Payload[CORE_TEMP_LENGTH];
//Data ID 0F
uint8_t Heartbeat_Payload[HEARTBEAT_LENGTH];

//SeqNum is [0,255]
uint8_t BBMB_SeqNum = 0;
uint8_t PPTMB_SeqNum = 0;
uint8_t MCMB_SeqNum = 0;
uint8_t DCMB_SeqNum = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* DATA ASSUMPTIONS:
 * Temp Range: 20-60
 * Max Temp Range: 20-30
 * Min Temp Range: 40-60
 * Reference: Aymon
 *
 * Current Range: 0-2 (6 digits after decimal)
 * Reference: Visual inspection from GenX test telemetry data
 *
 * Voltage Range: 2-5 (6 digits after decimal)
 * Reference: Visual inspection from GenX test telemetry data
 *
 * Bus_Metrics_Generator:
 *   samples of aggregate(0x03): 0x01
 * Reference: GenX PPTMB -> main.c -> static void adcTask(const void *pv);
 *
 * PPT_Metrics_Generator:
 *   PPT numbers: 7
 *   # of Sample Aggregate: 0x01
 * Reference: GenX PPTMB -> main.c -> static void adcTask(const void *pv);
 *
 * Speed_Pulse_Reading_And_MOT_LED_State_Generator:
 *   MOT_LED state: 0-1
 *   Speed_pulse Hz(uint16_t count): 0-255
 * Reference: GenX MCMB -> main.c -> static void spdTmr(TimerHandle_t xTimer);
 *
 * MC2_State_Generator:
 *   5 digital Buttons: 00000-11110 = 0-30
 *   Acc Pot (outputVal): 0 or 0xff
 * Reference: GenX DCMB -> main.c -> static void mc2StateTmr(TimerHandle_t xTimer);
 *
 * BBox_Startup_Generator:
 *   State (ignition_state): 0-1
 * Reference: GenX DCMB -> main.c -> void ignition_check(uint8_t data);
 *
 * PPTBox_Startup_Generator:
 *   State (array_state): 0-1
 * Reference: GenX DCMB -> main.c -> void ignition_check(uint8_t data);
 *
 * Light_State_Generator:
 *   left, right, brake: 0-1
 * Reference: GenX DCMB -> main.c -> static void lightsTmr(TimerHandle_t xTimer);
 *
 * Horn_State_Generator:
 *   State (horn_on): 0-1
 * Reference: GenX DCMB -> main.c -> static void buttonCheck(uint8_t state);
 * */

//Generate a random value in [max, min]
//One byte can express number 0-255, thus max<=255 && min>=0 && max>min
uint8_t getRandomValue(int min, int max){
    uint8_t r = rand() % (max + 1 - min) + min;
    return r;
}

//Insert num of random value in p starting at p[startPos]
void insertRandomValue(uint8_t* p, int startPos, int num, int min, int max){
    for(int i = startPos; i<startPos+num; i++){
        uint8_t r = getRandomValue(min, max);
        *(p+i) = r;
    }
}

float getRandomFloat(int min, int max) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = max - min;
    float r = random * diff;
    return min + r;
}

//Data ID 00
//Data ID 00
void Bus_Metrics_Generator(uint8_t data_ID, uint8_t* p){
    *p = data_ID;
    float Voltage_min = 100;  // random max and min
    float Voltage_max = 120;
    float Current_min = 10;  // random max and min
    float Current_max = 20;
    *(p+1) = 0x00;
    *(p+2) = 0x00;
    *(p+3) = 0x01;
    floatToArray(getRandomFloat(Voltage_min, Voltage_max), p+4);
    floatToArray(getRandomFloat(Current_min, Current_max), p+8);

}
void MC2_State_Generator(uint8_t* p){
	float min = 3;  // random max and min
	float max = 4;
	p[0] = DCMB_MC2_STATE_ID;
	p[1] = getRandomValue(0, 6); // moduleID
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
}
//Data ID 01
//void Cell_Metrics_Generator(uint8_t* p, uint8_t cellNum){
//	*(p) = 0x01;
//	*(p+1) = cellNum;
//	*(p+2) = 0x00;
//	*(p+3) = 0x01;
//	insertRandomValue(p, 4, 16, 0, 255);
//}
void PPT_Metrics_Generator(uint8_t* p){ //double check max and min values
	float Voltage_min = 0;  // random max and min for voltage
	float Voltage_max = 0;
	float Current_min = 0;  // random max and min for current
	float Current_max = 0;
	*p = PPTMB_PPT_METRICS_ID;
	p[1] = 0;
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(Voltage_min, Voltage_max), p+4);
	floatToArray(getRandomFloat(Current_min, Current_max), p+8);
	floatToArray(getRandomFloat(Voltage_min, Voltage_max), p+12);
	floatToArray(getRandomFloat(Current_min, Current_max), p+16);
	floatToArray(getRandomFloat(Voltage_min, Voltage_max), p+20);
	floatToArray(getRandomFloat(Current_min, Current_max), p+24);
}
void Speed_Pulse_Reading_Generator(uint8_t* p){
    *p = MCMB_CAR_SPEED_ID;
    insertRandomValue(p, 1, 1, 75, 80); //cruise speed 75-80
    *(p+2) = 0x00;
    *(p+3) = 0x00;
}
void BBox_Startup_Generator(uint8_t* p){
    *p = DCMB_BBOX_STARTUP_ID;
    insertRandomValue(p, 1, 1, 0, 1);
    *(p+2) = 0x00;
    *(p+3) = 0x00;
}
//Data ID 02
void BSD_Generator(uint8_t* p){
    *p = BBMB_BSD_ID;
    insertRandomValue(p, 1, 3, 0, 255);
}
void Motor_Temperature_Generator(uint8_t* p){
    *p = MCMB_MOTOR_TEMPERATURE_ID;
    insertRandomValue(p, 1, 1, 0, 40);
    *(p+2) = 0x00;
    *(p+3) = 0x00;
}
void PPTBox_Startup_Generator(uint8_t* p){
    *p = DCMB_PPTBOX_STARTUP_ID;
    insertRandomValue(p, 1, 1, 0, 1);
    *(p+2) = 0x00;
    *(p+3) = 0x00;
}
//Data ID 03
void BMS_MCU_Status_Generator(uint8_t* p){
    *p = BBMB_BMS_MCU_STATUS_ID;
    float min = 3;  // random max and min
  	float max = 4;
  	p[1] = 0;
  	p[2] = 0;
  	p[3] = 0;
  	floatToArray(getRandomFloat(min, max), p+4);
  	floatToArray(getRandomFloat(min, max), p+8);
  	floatToArray(getRandomFloat(min, max), p+12);
    insertRandomValue(p, 1, 15, 0, 255);
}
void Light_State_Generator(uint8_t* p){
    *p = DCMB_LIGHTCONTROL_ID;
    insertRandomValue(p, 1, 3, 0, 1);
}
void Supp_Battery_Metric_Generator(uint8_t* p){
	float min = 3;  // random max and min
	float max = 4;
	*p = MCMB_SUPP_BATT_VOLTAGE_ID;
	p[1] = 0;
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
}
//Data ID 04
void Streeting_Wheel_Generator(uint8_t* p){
    *p = DCMB_STEERING_WHEEL_ID;
    insertRandomValue(p, 1, 1, 0, 1);
    *(p+2) = 0x00;
    *(p+3) = 0x00;
}
//Data ID 05
//void BMS_Data_request_Generator(uint8_t* p){
//	float min = 3;  // random max and min
//	float max = 4;
//	*p = BMS_Data_request_Generator_ID;
//	p[1] = 0;
//	p[2] = 0;
//	p[3] = 0;
//	floatToArray(getRandomFloat(min, max), p+4);
//}
void Motor_Control_State_Generator(uint8_t* p){
	float min = 3;  // random max and min
	float max = 4;
	*p = DCMB_MOTOR_CONTROL_STATE_ID;
	p[1] = 0;
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
	floatToArray(getRandomFloat(min, max), p+8);
}
//Data ID 06
void Relay_State_Generator(uint8_t data_ID, uint8_t* p){
	float min = 0;  // random max and min
	float max = 1;
	*p = PPTMB_RELAYS_STATE_ID;
	p[1] = 0;
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
}
//Data ID 07
void Pedal_Angle_Generator(uint8_t* p){
	float min = 0;  // random max and min
	float max = 1;
	*p = DCMB_PEDALS_ANGLE_ID;
	p[1] = 0;
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
	floatToArray(getRandomFloat(min, max), p+8);
}
void BMS_Cell_Temp_Generator(uint8_t* p) {
	float min = 20;
	float max = 100;
	p[0] = BMS_CELL_TEMP_ID;
	p[1] = getRandomValue(0, 6); // moduleID - representing one battery module
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
	floatToArray(getRandomFloat(min, max), p+8);
	floatToArray(getRandomFloat(min, max), p+12);
	floatToArray(getRandomFloat(min, max), p+16);
	floatToArray(getRandomFloat(min, max), p+20);
}


//Data ID 08
void Side_Panel_Generator(uint8_t* p){
    *p = DCMB_SIDE_PANEL_ID;
    insertRandomValue(p,1,3,0,1);
}

void BMS_Cell_Volt_Generator(uint8_t* p) {
	float min = 3;  // cell voltage is between 3 and 4, usually around 3.7
	float max = 4;
	p[0] = BMS_CELL_VOLT_ID;
	p[1] = getRandomValue(0, 6); // moduleID
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
	floatToArray(getRandomFloat(min, max), p+8);
	floatToArray(getRandomFloat(min, max), p+12);
	floatToArray(getRandomFloat(min, max), p+16);
	floatToArray(getRandomFloat(min, max), p+20);
}

//Data ID 09
void BMS_Cell_SOC_Generator(uint8_t* p) {
	float min = 0;  // State of Charge is a percentage
	float max = 100;
	p[0] = BMS_CELL_SOC_ID;
	p[1] = getRandomValue(0, 6); // moduleID
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
	floatToArray(getRandomFloat(min, max), p+8);
	floatToArray(getRandomFloat(min, max), p+12);
	floatToArray(getRandomFloat(min, max), p+16);
	floatToArray(getRandomFloat(min, max), p+20);
}

//Data ID 0B
void Text_String_Generator(uint8_t* p){
	float min = 0;  // random values
	float max = 1;
	p[0] = MCMB_LP_BUS_METRICS_ID;
	p[1] = 0;
	p[2] = 0;
	p[3] = 0;
	floatToArray(getRandomFloat(min, max), p+4);
}
//Data ID 0D
//void LP_Bus_Metrics_Generator(uint8_t* p){ //not used
//	float min = 0;  // random vals
//	float max = 10;
//	p[0] = BBMB_LP_BUS_METRICS_ID;
//	p[1] = 0;
//	p[2] = 0;
//	p[3] = 0;
//	floatToArray(getRandomFloat(min, max), p+4);
//	floatToArray(getRandomFloat(min, max), p+8);
//	floatToArray(getRandomFloat(min, max), p+12);
//	floatToArray(getRandomFloat(min, max), p+16);
//}
//Data ID 0E
void Core_Temp_Generator(uint8_t* p){
    *p = MCMB_CORE_TEMP_ID;
    insertRandomValue(p, 1, 3, 20, 60);
}
//Data ID 0F
void Heartbeat_Generator(uint8_t* p){
    *p = BBMB_HEARTBEAT_ID;
    *(p+1) = 0x00;
    *(p+2) = 0x00;
    *(p+3) = 0x00;
}

void dummySend(uint8_t payloadLength, uint8_t senderAddress, uint8_t* seqNum, uint8_t* payload){

	uint8_t buf[HEADER_LENGTH + MAX_PACKET_SIZE + CRC_LENGTH];

	buf[0] = BSSR_SERIAL_START;
	buf[1] = payloadLength;
	buf[2] = senderAddress;
	buf[3] = *seqNum;
	memcpy(buf+4, payload, payloadLength);
	uint32_t crc_result = ~HAL_CRC_Calculate(&hcrc, (uint32_t*)buf, payloadLength+4);

	uint16_t buf_pos = 0;

	buf[buf_pos] = BSSR_SERIAL_START;
	buf_pos++;

	if(payloadLength == BSSR_SERIAL_START || payloadLength == BSSR_SERIAL_ESCAPE){
		buf[buf_pos] = BSSR_SERIAL_ESCAPE;
		buf_pos++;
		buf[buf_pos] = payloadLength;
		buf_pos++;
	} else{
		buf[buf_pos] = payloadLength;
		buf_pos++;
	}

	buf[buf_pos] = senderAddress;
	buf_pos++;

	if(*seqNum == BSSR_SERIAL_START || *seqNum == BSSR_SERIAL_ESCAPE){
		buf[buf_pos] = BSSR_SERIAL_ESCAPE;
		buf_pos++;
		buf[buf_pos] = *seqNum;
		buf_pos++;
	} else{
		buf[buf_pos] = *seqNum;
		buf_pos++;
	}
	(*seqNum)++;

	for(int i=0; i<payloadLength; i++){
		if(*(payload+i) == BSSR_SERIAL_ESCAPE || *(payload+i) == BSSR_SERIAL_START){
			buf[buf_pos] = BSSR_SERIAL_ESCAPE;
			buf_pos++;
			buf[buf_pos] = *(payload+i);
			buf_pos++;
		} else{
			buf[buf_pos] = *(payload+i);
			buf_pos++;
		}
	}

	for(int i=0; i<4; i++){
		uint8_t crc = (crc_result>>(8*(3-i))) & 255;
		if(crc == BSSR_SERIAL_ESCAPE || crc == BSSR_SERIAL_START){
			buf[buf_pos] = BSSR_SERIAL_ESCAPE;
			buf_pos++;
			buf[buf_pos] = crc;
			buf_pos++;
		} else{
			buf[buf_pos] = crc;
			buf_pos++;
		}
	}

	if(buf_pos%4 != 0) {
	        int paddingNum = 4 - buf_pos % 4;
	        for (int i = paddingNum; i > 0; i--) {
	            buf[buf_pos] = 0x00;
	            buf_pos++;
	        }
	    }

	HAL_UART_Transmit(&huart2, buf, buf_pos, HAL_MAX_DELAY);
}



void BBMB(){


	Bus_Metrics_Generator(BBMB_BUS_METRICS_ID, payload);
	dummySend(BUS_METRICS_LENGTH, BBMB_ID, &BBMB_SeqNum, payload);

	Relay_State_Generator(BBMB_RELAYS_STATE_ID, payload);
	dummySend(RELAY_STATE_LENGTH, BBMB_ID, &BBMB_SeqNum, payload);

	Heartbeat_Generator(payload);
	dummySend(HEARTBEAT_LENGTH, BBMB_ID, &BBMB_SeqNum, payload);

	BMS_Cell_Temp_Generator(payload);
	dummySend(BMS_CELL_TEMP_LENGTH, BBMB_ID, &BBMB_SeqNum, payload);

	BMS_Cell_Volt_Generator(payload);
	dummySend(BMS_CELL_VOLT_LENGTH, BBMB_ID, &BBMB_SeqNum, payload);

	BMS_Cell_SOC_Generator(payload);
	dummySend(BMS_CELL_SOC_LENGTH, BBMB_ID, &BBMB_SeqNum, payload);

}

void PPTMB(){

	Bus_Metrics_Generator(PPTMB_BUS_METRICS_ID, payload);
	dummySend(BUS_METRICS_LENGTH, PPTMB_ID, &PPTMB_SeqNum, payload);

	PPT_Metrics_Generator(payload);
	dummySend(PPT_METRICS_LENGTH, PPTMB_ID, &PPTMB_SeqNum, payload);

	Relay_State_Generator(PPTMB_RELAYS_STATE_ID, payload);
	dummySend(RELAY_STATE_LENGTH, PPTMB_ID, &PPTMB_SeqNum, payload);

	Heartbeat_Generator(payload);
	dummySend(HEARTBEAT_LENGTH, PPTMB_ID, &PPTMB_SeqNum, payload);

}

void MCMB(){

    Bus_Metrics_Generator(MCMB_BUS_METRICS_ID, Bus_Metrics_Payload);
    dummySend(BUS_METRICS_LENGTH, MCMB_ID, &MCMB_SeqNum, Bus_Metrics_Payload);

    Speed_Pulse_Reading_Generator(Speed_Pulse_Reading_Payload);
    dummySend(SPEED_PULSE_READING_LENGTH, MCMB_ID, &MCMB_SeqNum, Speed_Pulse_Reading_Payload);

    Heartbeat_Generator(Heartbeat_Payload);
    dummySend(HEARTBEAT_LENGTH, MCMB_ID, &MCMB_SeqNum, Heartbeat_Payload);

}

void DCMB(){

//    BBox_Startup_Generator(BBox_Startup_Payload);
//    dummySend(BBOX_STARTUP_LENGTH, DCMB_ID, &DCMB_SeqNum, BBox_Startup_Payload);
//
//    PPTBox_Startup_Generator(PPTBox_Startup_Payload);
//    dummySend(PPTBOX_STARTUP_LENGTH, DCMB_ID, &DCMB_SeqNum, PPTBox_Startup_Payload);

    Light_State_Generator(Light_State_Payload);//there
    dummySend(LIGHT_STATE_LENGTH, DCMB_ID, &DCMB_SeqNum, Light_State_Payload);

    Motor_Control_State_Generator(Motor_Control_State_Payload);//there
    dummySend(MOTOR_CONTROL_STATE_LENGTH, DCMB_ID, &DCMB_SeqNum, Motor_Control_State_Payload);

    Relay_State_Generator(DCMB_RELAYS_STATE_ID, Relay_State);//there
    dummySend(RELAY_STATE_LENGTH, DCMB_ID, &DCMB_SeqNum, Relay_State);

    Heartbeat_Generator(Heartbeat_Payload);//there
    dummySend(HEARTBEAT_LENGTH, DCMB_ID, &DCMB_SeqNum, Heartbeat_Payload);

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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  srand(time(NULL));
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
	 BBMB();
	 PPTMB();
	 MCMB();
	 DCMB();
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
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
