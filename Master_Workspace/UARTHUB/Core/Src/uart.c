/*
 * uart.c
 *
 *  Created on: Sep 21, 2019
 *      Author: jamesliu
 */

#include "uart.h"
#include "string.h"

#define NUM_UARTS 5 //TODO linkedList
#define RX_DMA_BUFFER_SIZE 2048 // must be above baud/10000
#define RX_CIRC_BUF_SIZE 2048

#define TX_Q_LEN 64
#define RX_Q_LEN 128
#define BUART_USE_MALLOC


/* START PORT DEPENDENT REGION */
// THIS WILL BE REPLACED WITH A LINKED LIST EVENTUALLY
static UART_HandleTypeDef* huartTable[NUM_UARTS] = {NULL};
static B_uartHandle_t* buartTable[NUM_UARTS] = {NULL};
/*  END PORT DEPENDENT REGION */


void B_uartStart(B_uartHandle_t* buart, UART_HandleTypeDef* huart);
void B_uartWrite(B_uartHandle_t* buart, uint8_t* buf, size_t len);
size_t B_uartRead(B_uartHandle_t* buart, uint8_t* buf); //returns len
static void txTask(void*);
static void rxTask(void*);
void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart);
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart);


void B_uartStart(B_uartHandle_t* buart, UART_HandleTypeDef* huart){
	vPortEnterCritical();
	for(size_t i = 0; i <= NUM_UARTS; i++){ //TODO linkedList
		if(i == NUM_UARTS) configASSERT(NULL);
		if(!huartTable[i]){
			huartTable[i] = huart;
			buartTable[i] = buart;
			break;
		}else if(buartTable[i] == buart || huartTable[i] == huart) configASSERT(NULL);
	}
	vPortExitCritical();
	buart->huart = huart;
	buart->topFlag = 0;
	buart->head = buart->tail = 0;
	buart->rxBuf = NULL;
	buart->rxQ = xQueueCreate(RX_Q_LEN, sizeof(B_bufQEntry_t));
	buart->txQ = xQueueCreate(128, sizeof(B_bufQEntry_t));
	buart->txSem = xSemaphoreCreateBinary();
	xTaskCreate(txTask, "BUARTTxTask", 512, buart, 6, &buart->txTask);
	xTaskCreate(rxTask, "BUARTRxTask", 512, buart, 6, &buart->rxTask);
}

void B_uartWrite(B_uartHandle_t* buart, uint8_t* buf, size_t len){
#ifdef BUART_USE_MALLOC
	uint8_t* mBuf = pvPortMalloc(len);
	memcpy(mBuf, buf, len);
	buf = mBuf;
#endif
	B_bufQEntry_t e;
	e.buf = buf;
	e.len = len;
	xQueueSendToBack(buart->txQ, &e, 0);

	/*	B_bufQEntry_t e;
	e.buf = pvPortMalloc(len);
	memcpy(e.buf, buf, len);
	e.len = len;
	int sent = xQueueSendToBack(buart->txQ, &e, 0);
	return sent;*/
}

size_t B_uartRead(B_uartHandle_t* buart, uint8_t* buf){
	B_bufQEntry_t e;
	xQueueReceive(buart->rxQ, &e, portMAX_DELAY);
	memcpy(buf, e.buf, e.len);
#ifdef BUART_USE_MALLOC
	vPortFree(e.buf);
#endif
	return e.len;
}


/*static void txTask(void* pv){
	B_uartHandle_t* buart = pv;
	B_bufQEntry_t e;
	for(;;){
		xQueueReceive(buart->txQ, &e, portMAX_DELAY);
		HAL_UART_Transmit_IT(buart->huart, e.buf, e.len);
		xSemaphoreTake(buart->txSem, portMAX_DELAY);
#ifdef BUART_USE_MALLOC
		vPortFree(e.buf);
#endif
	}
}*/
static void txTask(void* pv){
	B_uartHandle_t* buart = pv;
	B_bufQEntry_t e;

	//Temp
	//uint8_t buf[40];

	for(;;){
		xQueueReceive(buart->txQ, &e, portMAX_DELAY);

		//Temp
		//memcpy(&buf, e.buf, 40);

		HAL_UART_Transmit_DMA(buart->huart, e.buf, e.len);
		//HAL_UART_Transmit_IT(buart->huart, e.buf, e.len);

		//Waits until transmit is complete (happens when HAL_UART_TxCpltCallback is triggered)
		xSemaphoreTake(buart->txSem, portMAX_DELAY);
		//xSemaphoreTake(buart->txSem, pdMS_TO_TICKS(1000));

#ifdef BUART_USE_MALLOC
		vPortFree(e.buf);
#endif
	}
}

/*static void rxTask(void* pv){
	B_uartHandle_t* buart = pv;
	B_bufQEntry_t e;
#ifdef BUART_USE_MALLOC
	uint8_t* buf = pvPortMalloc(RX_DMA_BUFFER_SIZE);
#else
	uint8_t buf[RX_DMA_BUFFER_SIZE];
	uint8_t bufCpy[RX_DMA_BUFFER_SIZE];
#endif
//	buart->head = buart->tail = buf;
	configASSERT(HAL_UART_Receive_DMA(buart->huart, buf, RX_DMA_BUFFER_SIZE) == HAL_OK);
	//TickType_t tick = xTaskGetTickCount();
	for(;;){
		size_t bytesToRead;

		vPortEnterCritical();
		// no flags will update in this region. Capture head value at beginning.
		//buart->head = RX_DMA_BUFFER_SIZE - buart->huart->hdmarx->Instance->CNDTR; // 0 to MAX-1, cuz CNDTR is MAX to 1 in circular mode

		//temp
		buart->head = RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(buart->huart->hdmarx);

		// head is address TO BE written next, tail is address TO BE read next
		if(buart->topFlag){
			bytesToRead = RX_DMA_BUFFER_SIZE - buart->tail;
			buart->topFlag = 0;
		}else if(buart->head > buart->tail){
			bytesToRead = buart->head - buart->tail;
		}
		vPortExitCritical();

#ifdef BUART_USE_MALLOC
		uint8_t* bufCpy = pvPortMalloc(bytesToRead);
#endif
		memcpy(bufCpy, (uint8_t*)buf+buart->tail, bytesToRead);
		buart->tail = (buart->tail+bytesToRead) % RX_DMA_BUFFER_SIZE;

		e.buf = bufCpy;
		e.len = bytesToRead;
		xQueueSendToBack(buart->rxQ, &e, 0);
		//vTaskDelayUntil(&tick, 1);
		vTaskDelay(1);

	}
}*/

static void rxTask(void* pv){
	B_uartHandle_t* buart = pv;
	B_bufQEntry_t e;
	while(buart->huart->RxState != HAL_UART_STATE_READY) vTaskDelay(1);
	buart->rxBuf = pvPortMalloc(RX_CIRC_BUF_SIZE);
	while(!buart->rxBuf){
		vTaskDelay(1);
		buart->rxBuf = pvPortMalloc(RX_CIRC_BUF_SIZE);
	}
	configASSERT (HAL_UART_Receive_DMA(buart->huart, buart->rxBuf, RX_CIRC_BUF_SIZE) == HAL_OK);
	for(;;){
		e.len = 0;
		vPortEnterCritical();
		// no flags will update in this region. Capture head value at beginning.
		// 0 to MAX-1, cuz CNDTR is MAX to 1 in circular mode
		buart->head = RX_CIRC_BUF_SIZE - __HAL_DMA_GET_COUNTER(buart->huart->hdmarx);
		if(buart->topFlag){
			if(buart->head > buart->tail) buart->tail = buart->head;
			e.len = RX_CIRC_BUF_SIZE - buart->tail;
			buart->topFlag = 0;
		}else if(buart->head > buart->tail){
			e.len = buart->head - buart->tail;
		}
		vPortExitCritical();
		if(e.len){
			e.buf = pvPortMalloc(e.len);
			memcpy(e.buf, buart->rxBuf+buart->tail, e.len);
			buart->tail += e.len;
			buart->tail %= RX_CIRC_BUF_SIZE;
			int sent = xQueueSendToBack(buart->rxQ, &e, 0);
			//if(sent != pdTRUE) processCriticalFrame(&e);
		}
		vTaskDelay(1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	for(size_t i = 0; i < NUM_UARTS; i++){ //TODO linkedList
		if(huart == huartTable[i]){
			xSemaphoreGiveFromISR(buartTable[i]->txSem, NULL);
			return;
		}
	}
	configASSERT(NULL);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	for(size_t i = 0; i < NUM_UARTS; i++){ //TODO linkedList
		if(huart == huartTable[i]){
			buartTable[i]->topFlag = 1;
			return;
		}
	}
	configASSERT(NULL);
}
