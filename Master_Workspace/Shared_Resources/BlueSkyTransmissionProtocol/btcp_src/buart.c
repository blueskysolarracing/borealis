#include "buart.h"
#include "string.h"
#include "stdint.h"

#define NUM_UARTS 4
#define RX_CIRC_BUF_SIZE 2048
#ifdef BUART_INTERRUPT_MODE
#define BUART_IT_RX_BUF_SIZE 30
#define RX_QUEUE_SIZE 5
#else
#define RX_QUEUE_SIZE 64
#endif
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

// ########  ######## ########
// ##     ## ##       ##     ##
// ##     ## ##       ##     ##
// ########  ######   ########
// ##        ##       ##
// ##        ##       ##
// ##        ##       ##

//B_uartHandle_t* B_uartStart(UART_HandleTypeDef* huart);
//int B_uartSend(B_uartHandle_t* buart, uint8_t* buf, size_t len);
//B_bufQEntry_t* B_uartRead(B_uartHandle_t* buart);
//void B_uartDoneRead(B_bufQEntry_t* e);
//

static int mBuf_isEmpty(MsgBuf* m);
static int mBuf_isFull(MsgBuf* m);
static void mBuf_init(MsgBuf* m);
static int mBuf_getSize(MsgBuf* m );


static void txTask(void* pv);
static void rxTask(void* pv);
static void processCriticalFrame(B_bufQEntry_t* e);

//
//void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart);
//void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart);

// ######## ##     ## ##    ##  ######
// ##       ##     ## ###   ## ##    ##
// ##       ##     ## ####  ## ##
// ######   ##     ## ## ## ## ##
// ##       ##     ## ##  #### ##
// ##       ##     ## ##   ### ##    ##
// ##        #######  ##    ##  ######

B_uartHandle_t* B_uartStart(UART_HandleTypeDef* huart){
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
	BaseType_t taskcreate;
	taskcreate = xTaskCreate(txTask, "uartTxTask", TRX_TASK_STACK_SIZE, buart, TX_TASK_PRIORITY, &buart->txTask);
	configASSERT(taskcreate);

#ifdef BUART_INTERRUPT_MODE
	buart->rxQ = xQueueCreate(RX_QUEUE_SIZE, BUART_IT_RX_BUF_SIZE);
	while(huart->RxState != HAL_UART_STATE_READY) HAL_Delay(1);
	HAL_UART_Receive_IT(huart, buart->itBuf, BUART_IT_RX_BUF_SIZE);
	buart->itCallbackFlag = 0;

#else
	buart->rxQ = xQueueCreate(RX_QUEUE_SIZE, sizeof(B_bufQEntry_t));
	taskcreate = xTaskCreate(rxTask, "uartRxTask", TRX_TASK_STACK_SIZE, buart, RX_TASK_PRIORITY, &buart->rxTask);
	configASSERT(taskcreate);
	buart->topFlag = buart->head = buart->tail = 0;
#endif
	
	mBuf_init(&buart->mBuf); //For B_uartReadFullMessage
	return buart;
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
#ifdef BUART_INTERRUPT_MODE
	e->buf = pvPortMalloc(BUART_IT_RX_BUF_SIZE);
	xQueueReceive(buart->rxQ, e->buf, portMAX_DELAY);
	e->len = BUART_IT_RX_BUF_SIZE;
#else

	xQueueReceive(buart->rxQ, e, portMAX_DELAY);
#endif
	return e;
}

void B_uartDoneRead(B_bufQEntry_t* e){
	vPortFree(e->buf);
	vPortFree(e);
}

// Helpers for B_uartReadFullMessage()
static int mBuf_isEmpty(MsgBuf* m) {
    return (m->in == m->out);
}

static int mBuf_isFull(MsgBuf* m) {
    return ((m->in - m->out + m->maxLen) % (m->maxLen) == m->maxLen - 1);
}

static void mBuf_init(MsgBuf* m) {
	m->maxLen = MAX_BUART_MESSAGE_LENGTH;
	m->in = 0;
	m->out = 0;

}

static int mBuf_getSize(MsgBuf* m ){
	return (m->in - m->out + m->maxLen) % (m->maxLen);
}
	

// returns 1 if expectedLen is read
// returns 0 if expectedLen is too large (rxBuf will not be filled up)
int B_uartReadFullMessage(B_uartHandle_t* buart, uint8_t* rxBuf, uint8_t expectedLen, uint8_t startByteID) {

	MsgBuf* mBuf = &(buart->mBuf);
	uint8_t startFound = 0;
	uint8_t read = 0; //1 means has called B_uartRead, 0 means has not
	B_bufQEntry_t *e;
	do {
	        while (!mBuf_isEmpty(mBuf)) {
	        	// search for start byte in mBuf
	        	uint8_t s = mBuf->buf[mBuf->out];
	            if (s == startByteID) {
	                startFound = 1;
	                break; // Note we don't update 'out'. So, 'out' will index start
	            }
	            mBuf->out = (mBuf->out + 1) % mBuf->maxLen;
	        }
	        if (!startFound) {
	        	if (read) {
	        		B_uartDoneRead(e);
	        		read = 0;
	        	}
	        	e = B_uartRead(buart);
	        	read = 1;
	            for (int i = 0; i < e->len; i++) {
	                if (mBuf_isFull(mBuf)) break;
	                // put e->buf into mBuf
	                mBuf->buf[mBuf->in] = e->buf[i];
	                mBuf->in = (mBuf->in + 1) % mBuf->maxLen;
	            }
	        }
	    } while (!startFound);

	// fill up mBuf to expectedLen
	while (mBuf_getSize(mBuf) < expectedLen) {
		if (read) {
			B_uartDoneRead(e);
			read = 0;
		}
		e = B_uartRead(buart);
		read = 1;
		for (int i = 0; i < e->len; i++) {
			if (mBuf_isFull(mBuf)) {
		        B_uartDoneRead(e);
				return 0;
			}
			// put e->buf into mBuf
			mBuf->buf[mBuf->in] = e->buf[i];
			mBuf->in = (mBuf->in + 1) % mBuf->maxLen;
		}
	}

	// fill up rxBuf using mBuf
	for (int i = 0; i < expectedLen; i++) {
		rxBuf[i] = mBuf->buf[mBuf->out];
		mBuf->out = (mBuf->out + 1) % mBuf->maxLen;
	}

	if (read) {
		B_uartDoneRead(e);
		read = 0;
	}
	return 1;
}



static void txTask(void* pv){
	B_uartHandle_t* buart = pv;
	B_bufQEntry_t e;
	for(;;){
		xQueueReceive(buart->txQ, &e, portMAX_DELAY);
#ifdef BUART_INTERRUPT_MODE
		HAL_UART_Transmit_IT(buart->huart, e.buf, e.len);
#else
		HAL_UART_Transmit_DMA(buart->huart, e.buf, e.len);
#endif

		//Waits until transmit is complete (happens when HAL_UART_TxCpltCallback is triggered)
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
			/* ======= Testing begin ======= */
//			uint8_t tmp[e.len];
//			for (int i = 0; i < e.len; i++) {
//				tmp[i] = e.buf[i];
//			}
			/* ======== Testing end ========= */
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
#ifdef BUART_INTERRUPT_MODE
			buarts[i]->itCallbackFlag = 1;
#endif
			return;
		}
	}
	//configASSERT(NULL);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){

	for(size_t i = 0; i < NUM_UARTS; i++){ //TODO linkedList
		if(huart == huarts[i]){
#ifdef BUART_INTERRUPT_MODE
			xQueueSendToBackFromISR(buarts[i]->rxQ, buarts[i]->itBuf, 0);
			HAL_UART_Receive_IT(huart, buarts[i]->itBuf, BUART_IT_RX_BUF_SIZE);
#else
			buarts[i]->topFlag = 1;
#endif
			return;
		}
	}
	configASSERT(NULL);
}
