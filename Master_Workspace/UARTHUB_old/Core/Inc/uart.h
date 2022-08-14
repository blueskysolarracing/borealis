/*
 * uart.h
 *
 *  Created on: Sep 21, 2019
 *      Author: jamesliu
 */

#ifndef UART_H_
#define UART_H_

#include "main.h"
#include "cmsis_os.h"

// not __attribute__((__packed__))
typedef struct{
	UART_HandleTypeDef* 	huart;
	SemaphoreHandle_t 		txSem;
	QueueHandle_t 			txQ;
	uint8_t* 				rxBuf;
	QueueHandle_t 			rxQ;
	TaskHandle_t			txTask;
	TaskHandle_t			rxTask;
	size_t					head; // write end
	size_t					tail; // read end
	uint8_t					topFlag;
}B_uartHandle_t;

typedef struct{
	uint8_t* buf;
	size_t len;
}B_bufQEntry_t;

void B_uartStart(B_uartHandle_t* buart, UART_HandleTypeDef* huart);
void B_uartWrite(B_uartHandle_t* buart, uint8_t* buf, size_t len);
size_t B_uartRead(B_uartHandle_t* buart, uint8_t* buf); //returns len

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart);
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart);

#endif /* UART_H_ */
