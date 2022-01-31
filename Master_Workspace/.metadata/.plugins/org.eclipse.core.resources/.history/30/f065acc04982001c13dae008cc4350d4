#include "buart.h"
#include "string.h"
#include "SSD1322.h"
#include "main.h"

#define NUM_UARTS 4
#define RX_CIRC_BUF_SIZE 2048
#define RX_QUEUE_SIZE 64
#define TX_QUEUE_SIZE 64
#define TX_TASK_PRIORITY 5
#define RX_TASK_PRIORITY 6
#define TRX_TASK_STACK_SIZE 256

// ########  ##     ##
// ##     ## ##     ##
// ##     ## ##     ##
// ########  ##     ##
// ##         ##   ##
// ##          ## ##
// ##           ###

static B_uartHandle_t* buarts[NUM_UARTS];
static UART_HandleTypeDef* huarts[NUM_UARTS];
extern UART_HandleTypeDef huart2;
extern SemaphoreHandle_t txCpltSemHack;

// ########  ######## ########
// ##     ## ##       ##     ##
// ##     ## ##       ##     ##
// ########  ######   ########
// ##        ##       ##
// ##        ##       ##
// ##        ##       ##

B_uartHandle_t* B_uartStart(UART_HandleTypeDef* huart);
int B_uartSend(B_uartHandle_t* buart, uint8_t* buf, size_t len);
B_bufQEntry_t* B_uartRead(B_uartHandle_t* buart);
void B_uartDoneRead(B_bufQEntry_t* e);
static void txTask(void* pv);
static void rxTask(void* pv);
static void processCriticalFrame(B_bufQEntry_t* e);

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart);
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart);

// ######## ##     ## ##    ##  ######
// ##       ##     ## ###   ## ##    ##
// ##       ##     ## ####  ## ##
// ######   ##     ## ## ## ## ##
// ##       ##     ## ##  #### ##
// ##       ##     ## ##   ### ##    ##
// ##        #######  ##    ##  ######

int B_uartstart(UART_HandleTypeDef* huart){
	B_uartHandle_t *buart;
    for(int i = 0; i < 3; i++){
        if(buarts[i] == NULL){
            buarts[i] = pvPortMalloc(sizeof(B_uartHandle_t));
            buart = buarts[i];
            huarts[i] = huart;
            buart->huart = huart;
            break;
        }
    }
    buart->txSem = xSemaphoreCreateBinary();
	buart->txQ = xQueueCreate(TX_QUEUE_SIZE, sizeof(B_bufQEntry_t));
	// buart->rxBuf = pvPortMalloc(RX_CIRC_BUF_SIZE); // done in task
	buart->rxQ = xQueueCreate(RX_QUEUE_SIZE, sizeof(B_bufQEntry_t));
	xTaskCreate(txTask, "uartTxTask", TRX_TASK_STACK_SIZE, buart, TX_TASK_PRIORITY, &buart->txTask);
	xTaskCreate(rxTask, "uartTxTask", TRX_TASK_STACK_SIZE, buart, RX_TASK_PRIORITY, &buart->rxTask);
	buart->topFlag = buart->head = buart->tail = 0;
}

int B_uartSend(B_uartHandle_t* buart, uint8_t* buf, size_t len){
	B_bufQEntry_t e;
	e.buf = pvPortMalloc(len);
	memcpy(e.buf, buf, len);
	e.len = len;
	int sent = xQueueSendToBack(buart->txQ, &e, 0);
	return sent;
}

B_bufQEntry_t* B_uartRead(B_uartHandle_t* buart){
	B_bufQEntry_t* e = pvPortMalloc(sizeof(B_bufQEntry_t));
	xQueueReceive(buart->rxQ, e, portMAX_DELAY);
	return e;
}

void B_uartDoneRead(B_bufQEntry_t* e){
	vPortFree(e->buf);
	vPortFree(e);
}

//  ######  ########    ###    ######## ####  ######
// ##    ##    ##      ## ##      ##     ##  ##    ##
// ##          ##     ##   ##     ##     ##  ##
//  ######     ##    ##     ##    ##     ##  ##
//       ##    ##    #########    ##     ##  ##
// ##    ##    ##    ##     ##    ##     ##  ##    ##
//  ######     ##    ##     ##    ##    ####  ######

static void txTask(void* pv){
	B_uartHandle_t* buart = pv;
	B_bufQEntry_t e;
	for(;;){
		xQueueReceive(buart->txQ, &e, portMAX_DELAY);
		HAL_UART_Transmit_DMA(buart->huart, e.buf, e.len);
		xSemaphoreTake(buart->txSem, portMAX_DELAY);
		vPortFree(e.buf);
	}
}

static void rxTask(void* pv){
	B_uartHandle_t* buart = pv;
	B_bufQEntry_t e;
	while(buart->huart->RxState != HAL_UART_STATE_READY) vTaskDelay(1);
	buart->rxBuf = pvPortMalloc(RX_CIRC_BUF_SIZE);
	while(!buart->rxBuf){
		vTaskDelay(1);
		buart->rxBuf = pvPortMalloc(RX_CIRC_BUF_SIZE);
	}
	HAL_UART_Receive_DMA(buart->huart, buart->rxBuf, RX_CIRC_BUF_SIZE);
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
			if(sent != pdTRUE) processCriticalFrame(&e);
		}
		vTaskDelay(1);
	}
}

static void processCriticalFrame(B_bufQEntry_t* e){

}


//  ######     ###    ##       ##       ########     ###     ######  ##    ##
// ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
// ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
// ##       ##     ## ##       ##       ########  ##     ## ##       #####
// ##       ######### ##       ##       ##     ## ######### ##       ##  ##
// ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
//  ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##


void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	for(size_t i = 0; i < NUM_UARTS; i++){ //TODO linkedList
		if(huart == huarts[i]){
			xSemaphoreGiveFromISR(buarts[i]->txSem, NULL);
			return;
		}
	}
	if(huart == &huart2){
		xSemaphoreGiveFromISR(txCpltSemHack, NULL);
	}
	//configASSERT(NULL);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	for(size_t i = 0; i < NUM_UARTS; i++){ //TODO linkedList
		if(huart == huarts[i]){
			buarts[i]->topFlag = 1;
			return;
		}
	}
	configASSERT(NULL);
}
