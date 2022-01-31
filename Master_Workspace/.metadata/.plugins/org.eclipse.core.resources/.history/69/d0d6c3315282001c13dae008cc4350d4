#ifndef _MY_TASKS_H__
#define _MY_TASKS_H__

#define CAN_Q_LENGTH 10
#define CAN_ITEM_SIZE 8
#define BSSR_CAN_TX_ID 0x11
#define BSSR_CAN_RX_ID 0x11
#define BSSR_CAN_RX_MASK 0
#define BSSR_CAN_WATERMARK_LEVEL 16

/** TASK priorities */
#define CAN_QUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	CAN_QUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/** TASK intervals */
#define CAN_TRANSMIT_TASK_INTERVAL   100 /* ms */

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32h7xx_hal.h"
#include "btcp.h"
extern int f_can_status;

void BSSR_CAN_TEST(FDCAN_HandleTypeDef * hfdcan);

/** start the can bus 
 * This function will only run after can initialization and only once
 **/
void BSSR_CAN_START();

void BSSR_CAN_Tx(char * data);

/** A task to handle the transmition */
void BSSR_CAN_TxTask(void * pvParameters);
void BSSR_CAN_RxTask(void * pvParameters);
void BSSR_CAN_TASK_INIT(FDCAN_HandleTypeDef * hfdcan, UART_HandleTypeDef * huart2, B_tcpHandle_t *btcpS);
void BSSR_CAN_TASK_INIT2(FDCAN_HandleTypeDef * hfdcan);
void BSSR_CAN_testTask(void * pvParameters);

#endif // !_MY_TASKS_H__
