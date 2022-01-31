#include "main.h"
#include "cmsis_os.h"

#ifndef __BUART_H
#define __BUART_H
typedef struct{
	uint8_t* buf;
	size_t len;
}B_bufQEntry_t;

// not __attribute__((__packed__))
typedef struct{
	UART_HandleTypeDef* 	huart;
	SemaphoreHandle_t 		txSem;
	QueueHandle_t 			txQ;
	uint8_t* 				rxBuf;
	QueueHandle_t 			rxQ;
	TaskHandle_t			txTask;
	TaskHandle_t			rxTask;
	// head is address TO BE written next, tail is address TO BE read next
	size_t					head;
	size_t					tail;
	uint8_t					topFlag;
}B_uartHandle_t;

int B_uartSend(B_uartHandle_t* buart, uint8_t* buf, size_t len);
B_bufQEntry_t* B_uartRead(B_uartHandle_t* buart);
void B_uartDoneRead(B_bufQEntry_t* e);

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart);
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart);
#endif

#ifndef __BUART_START_H
#define __BUART_START_H
B_uartHandle_t* B_uartStart(UART_HandleTypeDef* huart);
#endif
