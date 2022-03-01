#include "BSSR_CAN_H7.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "btcp.h"
#define DEBUG_ON

#define MAX_TEMP 333000
#define MIN_TEMP 0
#define MIN_VOLTAGE 0
#define MAX_VOLTAGE 4200

static FDCAN_FilterTypeDef filterConfig;
static FDCAN_TxHeaderTypeDef txHeader;
static FDCAN_RxHeaderTypeDef rxHeader;
static FDCAN_TxEventFifoTypeDef txEvtHeader;
static FDCAN_HandleTypeDef * fdcan;
static UART_HandleTypeDef * uart;
static uint32_t dataCnt;

static QueueHandle_t xCanTxQueue = NULL;
static QueueHandle_t xCanRxQueue = NULL;

static char msg[CAN_ITEM_SIZE + 1] = {0};
static char buffer[100];
static int boardId, cellId, voltage, temp;
B_tcpHandle_t *btcp;
static volatile double fake_rand() {
    static int seed = 1;
    const static int a = 1627, b = 2549, m = 10000;
    seed = (seed * a + b) % m;
    return seed / m;
}
typedef struct {
	uint16_t maxVoltage;
	uint16_t maxTemp;
	uint16_t minVoltage;
	uint16_t minTemp;
	uint16_t numSamples;
	uint32_t aggTemp;
	uint32_t aggVoltage;
	SemaphoreHandle_t cellSem;
	uint8_t fucky;
}CellInfo_t;

CellInfo_t cells[30];
static void BSSR_CAN_Error(char *msg, FDCAN_HandleTypeDef * hfdcan) {
    char buffer[128];
    sprintf(buffer, "CAN TASK: %s\r\n", msg);
    //HAL_UART_Transmit(uart, buffer, strlen(buffer), 200);
    if (hfdcan != NULL) {
        sprintf(buffer, "\tERROR CODE: 0x%x\r\n", HAL_FDCAN_GetError(hfdcan));
       // HAL_UART_Transmit(uart, buffer, strlen(buffer), 200);
    }
#ifdef DEBUG_ON
//    for (;;);
#endif
}

static inline void BSSR_CAN_Log(char *msg) {
#ifdef DEBUG_ON
    char buffer[128];
    sprintf(buffer, "CAN TASK Message: %s\r\n", msg);
    //HAL_UART_Transmit(uart, buffer, strlen(buffer), 1000);
#endif
}

static inline void BSSR_CAN_Status_Log(FDCAN_HandleTypeDef * hfdcan) {
#ifdef DEBUG_ON
    switch (HAL_FDCAN_GetState(hfdcan)) {
        case HAL_FDCAN_STATE_RESET:
            BSSR_CAN_Log("HAL_FDCAN_STATE_RESET");
            break;
        case HAL_FDCAN_STATE_READY:
            BSSR_CAN_Log("HAL_FDCAN_STATE_READY");
            break;  
        case HAL_FDCAN_STATE_BUSY:
            BSSR_CAN_Log("HAL_FDCAN_STATE_BUSY");
            break;
        case HAL_FDCAN_STATE_ERROR:
            BSSR_CAN_Log("HAL_FDCAN_STATE_ERROR");
            break;
        default:
            return;
    }
#endif // DEBUG
}
static void writeCell(uint8_t *buf, uint8_t cell_id){
	CellInfo_t *cell = &(cells[cell_id]);
	xSemaphoreTake(cell->cellSem, 1);
	buf[1] = cell_id;
	buf[2] = (cell->numSamples >> 8) & 0xff;
	buf[3] = cell->numSamples & 0xff;
	cell->numSamples = 0;
	buf[4] = (cell->aggTemp >> 24) & 0xff;
	buf[5] = (cell->aggTemp >> 16) & 0xff;
	buf[6] = (cell->aggTemp >> 8) & 0xff;
	buf[7] = (cell->aggTemp) & 0xff;
	cell->aggTemp = 0;
	buf[8] = (cell->aggVoltage >> 24) & 0xff;
	buf[9] = (cell->aggVoltage >> 16) & 0xff;
	buf[10] = (cell->aggVoltage >> 8) & 0xff;
	buf[11] = (cell->aggVoltage) & 0xff;
	cell->aggVoltage = 0;
	buf[12] = (cell->maxTemp >> 8) & 0xff;
	buf[13] = (cell->maxTemp) &0xff;
	buf[14] = (cell->minTemp >> 8) &0xff;
	buf[15] = (cell->minTemp) &0xff;
	buf[16] = (cell->maxVoltage >> 8) &0xff;
	buf[17] = (cell->maxVoltage) &0xff;
	buf[18] = (cell->minVoltage >> 8) &0xff;
	buf[19] = (cell->minVoltage) &0xff;
	xSemaphoreGive(cells[cell_id].cellSem);
}

static void cellStateTmr(TimerHandle_t xTimer){
	uint8_t packet[200];
	//uint8_t packet_pos = 1;
	packet[0] = 1;
	for(int i = 0; i < 3; i++){
		for(int j = 0 + i*10; j < (i+1)*10; j++){
			writeCell(packet+(j*20), i*10 + j);
		}
		B_tcpSend(btcp, packet, 200);
	}
}
void BSSR_CAN_TASK_INIT(FDCAN_HandleTypeDef * hfdcan, UART_HandleTypeDef * huart2, B_tcpHandle_t *btcpS) {
    fdcan = hfdcan;
    btcp = btcpS;
    uart = huart2;
    BSSR_CAN_START();
    dataCnt = 0;
    // BSSR_CAN_Log("CAN TxTask Started!");
    // xTaskCreate(BSSR_CAN_TxTask, "CanTxTask", configMINIMAL_STACK_SIZE, NULL, CAN_QUEUE_SEND_TASK_PRIORITY, NULL);
    xTaskCreate(BSSR_CAN_RxTask, "CanRxTask", configMINIMAL_STACK_SIZE, NULL, CAN_QUEUE_RECEIVE_TASK_PRIORITY, NULL);
    xTimerStart(xTimerCreate("cellTmr", 150, pdTRUE, NULL, cellStateTmr), 0);

    for(int i = 0; i < 30; i++){
    	cells[i].aggTemp = 0;
    	cells[i].aggVoltage = 0;
    	cells[i].maxTemp = 0;
    	cells[i].maxVoltage = 0;
    	cells[i].minTemp = 0;
    	cells[i].minVoltage = 0;
    	cells[i].numSamples = 0;
    	cells[i].cellSem = xSemaphoreCreateMutex();

    }
    uint8_t fucky[1] = {4};
    for(int i = 0; i < 0; i++){
    	cells[i].fucky = 1;
    }


}

void BSSR_CAN_START() {

    // //1. Initialize the FDCAN peripheral using HAL_FDCAN_Init function; waiting for f_can_init flag
    // if (f_can_status == 0) return;
    // // set flag to false, so this functino wont run next time
    // f_can_status = 0;

    //0. Initialize the queue for can tx
    if ((xCanTxQueue = xQueueCreate(CAN_Q_LENGTH, CAN_ITEM_SIZE * sizeof(char))) == NULL) {
        //Error
        BSSR_CAN_Error("create xCanTxQueue", NULL);
    }
    if ((xCanRxQueue = xQueueCreate(CAN_Q_LENGTH, CAN_ITEM_SIZE * sizeof(char))) == NULL) {
        //Error
        BSSR_CAN_Error("create xCanRxQueue", NULL);
    }
    
    //2. Configure the reception filters and optional features
    filterConfig.IdType         = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex    = 0;
    filterConfig.FilterType     = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig   = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1      = BSSR_CAN_RX_ID;
    filterConfig.FilterID2      = BSSR_CAN_RX_MASK; /* For acceptance, MessageID and FilterID1 must match exactly */
    
    /* Prepare Tx Header */
    txHeader.Identifier         = BSSR_CAN_TX_ID;
    txHeader.IdType             = FDCAN_STANDARD_ID;
    txHeader.TxFrameType        = FDCAN_DATA_FRAME;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch      = FDCAN_BRS_OFF;
    txHeader.FDFormat           = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    txHeader.MessageMarker      = 0;

    // /* Tx Event Header */
    // txEvtHeader.Identifier      = BSSR_CAN_TX_ID;
    // txEvtHeader.IdType          = FDCAN_STANDARD_ID;
    // txEvtHeader.TxFrameType     = FDCAN_DATA_FRAME;
    // txEvtHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // txEvtHeader.BitRateSwitch   = FDCAN_BRS_OFF;
    // txEvtHeader.FDFormat        = FDCAN_CLASSIC_CAN;
    // txEvtHeader.MessageMarker   = 0;

    // /** Prepaer Rx Header */
    // rxHeader.Identifier         = 0;
    // rxHeader.IdType             = FDCAN_STANDARD_ID;
    // rxHeader.RxFrameType        = FDCAN_FRAME_CLASSIC;
    // rxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // rxHeader.BitRateSwitch      = FDCAN_BRS_OFF;
    // rxHeader.FDFormat           = FDCAN_CLASSIC_CAN;
    // rxHeader.IsFilterMatchingFrame = 1;

    // Set up bytes

#if CAN_ITEM_SIZE == 8
    txHeader.DataLength         = FDCAN_DLC_BYTES_8;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_8;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_8;
#elif CAN_ITEM_SIZE == 12
    txHeader.DataLength         = FDCAN_DLC_BYTES_12;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_12;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_12;
#elif CAN_ITEM_SIZE == 16
    txHeader.DataLength         = FDCAN_DLC_BYTES_16;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_16;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_16;
#elif CAN_ITEM_SIZE == 20
    txHeader.DataLength         = FDCAN_DLC_BYTES_20;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_20;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_20;
#elif CAN_ITEM_SIZE == 24
    txHeader.DataLength         = FDCAN_DLC_BYTES_24;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_24;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_24;
#elif CAN_ITEM_SIZE == 32
    txHeader.DataLength         = FDCAN_DLC_BYTES_32;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_32;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_32;
#elif CAN_ITEM_SIZE == 48
    txHeader.DataLength         = FDCAN_DLC_BYTES_48;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_48;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_48;
#else
    txHeader.DataLength         = FDCAN_DLC_BYTES_64;
    txEvtHeader.DataLength      = FDCAN_DLC_BYTES_64;
    rxHeader.DataLength         = FDCAN_DLC_BYTES_64;
#endif

    if (HAL_OK != HAL_FDCAN_ConfigFilter(fdcan, &filterConfig)) {
        // Error_HANDLER;
        BSSR_CAN_Error("HAL_FDCAN_ConfigFilter", NULL);
    };
    //Reject other data to FIFO
    if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE)) {
        //Error Handler
        BSSR_CAN_Error("HAL_FDCAN_ConfigGlobalFilter", NULL);
    }
    /* Configure Rx FIFO 0 watermark level to 2 */
    // if (HAL_OK != HAL_FDCAN_ConfigFifoWatermark(fdcan, FDCAN_CFG_RX_FIFO0, BSSR_CAN_WATERMARK_LEVEL)) {
    //     BSSR_CAN_Error("HAL_FDCAN_ConfigFifoWatermark FDCAN_CFG_RX_FIFO0", NULL);
    // }

    //3. Start the FDCAN module using HAL_FDCAN_Start function. 
    // At this level the node is active on the bus: it cansend and receive messages
    if (HAL_OK != HAL_FDCAN_Start(fdcan)) {
        // Error_HANDLER;
        BSSR_CAN_Error("HAL_FDCAN_Start", NULL);
    };

    /* Activate New Message Interrupt */
    // if (HAL_OK != HAL_FDCAN_ActivateNotification(fdcan,
    //         // FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | 
    //         FDCAN_IT_RX_FIFO0_FULL | 
    //         FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0)) {
    //     BSSR_CAN_Error("HAL_FDCAN_ActivateNotification FDCAN_IT_RX_*", NULL);
    // }

    if (HAL_OK != HAL_FDCAN_ActivateNotification(fdcan, 
            FDCAN_IT_TX_EVT_FIFO_ELT_LOST | FDCAN_IT_TX_EVT_FIFO_FULL | 
            FDCAN_IT_TX_EVT_FIFO_WATERMARK | FDCAN_IT_TX_EVT_FIFO_NEW_DATA |
            FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY, 0)) {
        BSSR_CAN_Error("HAL_FDCAN_ActivateNotification FDCAN_IT_TX_*", NULL);
    }

    if (HAL_OK != HAL_FDCAN_ActivateNotification(fdcan, 
            FDCAN_IT_RX_HIGH_PRIORITY_MSG|FDCAN_IT_RX_BUFFER_NEW_MESSAGE|
            FDCAN_IT_TIMESTAMP_WRAPAROUND|FDCAN_IT_TIMEOUT_OCCURRED|
            FDCAN_IT_CALIB_STATE_CHANGED|FDCAN_IT_CALIB_WATCHDOG_EVENT|
            
            FDCAN_IT_RX_FIFO1_MESSAGE_LOST|FDCAN_IT_RX_FIFO1_FULL|
            FDCAN_IT_RX_FIFO1_WATERMARK|FDCAN_IT_RX_FIFO1_NEW_MESSAGE|
            
            FDCAN_IT_RAM_ACCESS_FAILURE|FDCAN_IT_ERROR_LOGGING_OVERFLOW|
            FDCAN_IT_ERROR_PASSIVE|FDCAN_IT_ERROR_WARNING|FDCAN_IT_BUS_OFF|
            FDCAN_IT_RAM_WATCHDOG|FDCAN_IT_ARB_PROTOCOL_ERROR|
            FDCAN_IT_DATA_PROTOCOL_ERROR|FDCAN_IT_RESERVED_ADDRESS_ACCESS, 0)) {
        BSSR_CAN_Error("HAL_FDCAN_ActivateNotification ALL_*", NULL);
    }
    
    //4. Tx control functions
    // if (HAL_FDCAN_EnableTxBufferRequest(fdcan,FDCAN_TX_BUFFER0) != HAL_OK){
    //     BSSR_CAN_Error("HAL_FDCAN_EnableTxBufferRequest", NULL);
    // }
    char m[CAN_ITEM_SIZE] = "Start";
    // if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &txHeader, m)) {
    //     BSSR_CAN_Error("HAL_FDCAN_AddMessageToTxFifoQ", fdcan);
    // }

    BSSR_CAN_Log("CAN init DONE");
    BSSR_CAN_Status_Log(fdcan);

}

void BSSR_CAN_Tx(char * data) {
    char d[CAN_ITEM_SIZE];
    strncpy(d, data, CAN_ITEM_SIZE);    // avoid out of range problem
    d[CAN_ITEM_SIZE - 1] = 0;
    xQueueSendToBack(xCanTxQueue, d, 0);
}

void BSSR_CAN_TxTask(void * pvParameters) {
    char msg[CAN_ITEM_SIZE];
    for(;;) {
        //check whether the tx fifo q is able to push and our q is not empty
        while (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) > 0) {
            msg[0] = 0;
            xQueueReceive(xCanTxQueue, msg, 0);
            if (msg[0] == 0) break;

            if (HAL_OK != HAL_FDCAN_ActivateNotification(fdcan,
                    FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | 
                    FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0)) {
                BSSR_CAN_Error("HAL_FDCAN_ActivateNotification FDCAN_IT_RX_*", NULL);
            }

            if (HAL_OK != HAL_FDCAN_ActivateNotification(fdcan, 
                    FDCAN_IT_TX_EVT_FIFO_ELT_LOST | FDCAN_IT_TX_EVT_FIFO_FULL | 
                    FDCAN_IT_TX_EVT_FIFO_WATERMARK | FDCAN_IT_TX_EVT_FIFO_NEW_DATA |
                    FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY, 0)) {
                BSSR_CAN_Error("HAL_FDCAN_ActivateNotification FDCAN_IT_TX_*", NULL);
            }

            HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &txHeader, msg);
            // HAL_FDCAN_AddMessageToTxBuffer(fdcan, &txHeader, msg, FDCAN_TX_BUFFER0);
            // strcat(msg, "(sent)");

            // char buffer[100];
            // sprintf(buffer, "TxQ Free Level:0x%x", HAL_FDCAN_GetTxFifoFreeLevel(fdcan));
            // BSSR_CAN_Log(buffer);
            
            // sprintf(buffer, "RxQ Fill Level:0x%x", HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0));
            // BSSR_CAN_Log(buffer);
        }
        osDelay(100);
    }
}

void BSSR_CAN_RxTask(void * pvParameters) {
    BSSR_CAN_Log("RxTask started");
    for (;;) {
        if (HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0) > 1) {
            getTempVoltData(fdcan);
        }
        osDelay(0);
    }
}

void BSSR_CAN_testTask(void * pvParameters) {

    char msg[CAN_ITEM_SIZE] = {0};
    int a = -1;
    vTaskDelay(1000);
    for(;;) {
        txHeader.Identifier++;
        a = (a + 1) % 10;
        msg [0] = 'T';
        msg [1] = a + 48;
        msg [2] = ':';
        for (int i = 3; i < CAN_ITEM_SIZE - 1; i++) {
            msg[i] = 'a' + (a + i) % 26;
        }
        msg [CAN_ITEM_SIZE - 1] = 0;
        BSSR_CAN_Tx(msg);
        //osDelay(10);
        vTaskDelay(9970);
    }
}

void BSSR_CAN_TEST(FDCAN_HandleTypeDef * hfdcan) {
    xTaskCreate(BSSR_CAN_testTask, "CanTestTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
}

// TODO read data here
void getTempVoltData(FDCAN_HandleTypeDef *hfdcan) {
    if (HAL_OK == HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, msg)) {
        // BaseType_t taskWoken = pdFALSE;
        // xQueueSendToBackFromISR(xCanRxQueue, msg, &taskWoken);
        osDelay(0);
        
        boardId = rxHeader.Identifier >> 3;
        cellId = rxHeader.Identifier & 0x07;
        voltage = *(int *)(msg+0);
        temp = *(int *)(msg+4);

        // filter out wrong data
        uint8_t cellNum = (boardId-1)*5 + cellId;
        xSemaphoreTake(cells[cellNum].cellSem, 1);
        cells[cellNum].aggTemp += temp;
        cells[cellNum].aggVoltage += voltage;
        cells[cellNum].numSamples++;
        xSemaphoreGive(cells[cellNum].cellSem);
        if (!(voltage > 7000 || voltage < 2200 || temp > 373000 || temp < 220000)) {

           // sprintf(buffer, "MSG ID=0o%04o, CONTENT=0x%02x%02x%02x%02x%02x%02x%02x%02x, boardId=%d, cellId=%d, voltage=%d, temp=%d",
//                rxHeader.Identifier,
//                *(uint8_t *)(msg+7), *(uint8_t *)(msg+6), *(uint8_t *)(msg+5), *(uint8_t *)(msg+4),
//                *(uint8_t *)(msg+3), *(uint8_t *)(msg+2), *(uint8_t *)(msg+1), *(uint8_t *)(msg+0),
//                boardId, cellId, voltage, (temp-273150));
            if(voltage > MAX_VOLTAGE || voltage < MIN_VOLTAGE || temp > MAX_TEMP || temp < MIN_TEMP){
                // Trigger BSD
                HAL_GPIO_WritePin(GPIOI, GPIO_PIN_13, GPIO_PIN_RESET);
                uint8_t buf[4] = {0x02, 0x00, 0x00, 0x00};
                if(voltage > MAX_VOLTAGE){
                	buf[1] = 0x01;
                	buf[2] = (voltage >> 8) &0xff;
                	buf[3] = voltage &0xff;
                } else if (voltage < MIN_VOLTAGE){
                	buf[1] = 0x02;
                	buf[2] = (voltage >> 8) &0xff;
                	buf[3] = voltage &0xff;
                } else if (temp > MAX_TEMP){
                	buf[1] = 0x03;
                	buf[2] = (temp >> 8) &0xff;
                	buf[3] = temp & 0xff;
                } else if (temp < MIN_TEMP){
                	buf[1] = 0x04;
                	buf[2] = (temp >> 8) &0xff;
                	buf[3] = temp & 0xff;
                }
                B_tcpSend(btcp, buf, 4);
                //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
                //HAL_UART_Transmit_IT(uart, buffer, strlen(buffer));
                //HAL_UART_Transmit_IT(uart, "CAN TRIPPED\r\n", strlen("CAN TRIPPED\r\n"));

            }
            //BSSR_CAN_Log(buffer);
        } else {
            //BSSR_CAN_Log("Error in Reading");
        }
    }
}

/**
 * Callback for RxFifo0
 * */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    BSSR_CAN_Log("RxFifo0Callback");
    // BSSR_CAN_Status_Log(hfdcan);

    // if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) != RESET)
    // {
    //     /* Retreive Rx messages from RX FIFO0 */
    //     HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, msg);

        
        
    //     /* Activate Rx FIFO 0 watermark notification */
    //     HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
        
    // }
    
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) 
        BSSR_CAN_Log("FDCAN_IT_RX_FIFO0_MESSAGE_LOST");
    if (RxFifo0ITs &  FDCAN_IT_RX_FIFO0_FULL)
        BSSR_CAN_Log("FDCAN_IT_RX_FIFO0_FULL");
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK)
        BSSR_CAN_Log("FDCAN_IT_RX_FIFO0_WATERMARK");
    // if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    //     BSSR_CAN_Log("FDCAN_IT_RX_FIFO0_NEW_MESSAGE");

    // // HAL_FDCAN_GetRxMessage
    // if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK || RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) {
    //     for (int i = 0; i < BSSR_CAN_WATERMARK_LEVEL; i++) getTempVoltData(hfdcan);
    // }

    if (HAL_OK != HAL_FDCAN_ActivateNotification(fdcan,
            // FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | 
            FDCAN_IT_RX_FIFO0_FULL | 
            FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0)) {
        BSSR_CAN_Error("HAL_FDCAN_ActivateNotification FDCAN_IT_RX_*", NULL);
    }
}

void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan) {
    // HAL_FDCAN_GetRxMessage(hfdcan)
        BSSR_CAN_Log("RxBufferNewMessageCallback");
        
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
    BSSR_CAN_Log("RxFifo1Received");
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs) {

    BSSR_CAN_Log("TxEventFifoCallback");
    HAL_FDCAN_GetTxEvent(hfdcan, &txEvtHeader);

    if (TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_ELT_LOST)
        BSSR_CAN_Log("FDCAN_IT_TX_EVT_FIFO_ELT_LOST");
    if (TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_FULL)
        BSSR_CAN_Log("FDCAN_IT_TX_EVT_FIFO_FULL");
    if (TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_WATERMARK)
        BSSR_CAN_Log("FDCAN_IT_TX_EVT_FIFO_WATERMARK");
    if (TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_NEW_DATA)
        BSSR_CAN_Log("FDCAN_IT_TX_EVT_FIFO_NEW_DATA");
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
    
    BSSR_CAN_Log("TxFifoEmptyCallback");
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
    BSSR_CAN_Log("HAL_FDCAN_TxBufferCompleteCallback");
}
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
    BSSR_CAN_Log("HAL_FDCAN_TxBufferAbortCallback");
}



void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
    BSSR_CAN_Error("unknown", hfdcan);
}

// NOTE UNUSED callbacks

void HAL_FDCAN_TimeoutOccurredCallback(FDCAN_HandleTypeDef *hfdcan) {
    BSSR_CAN_Log("HAL_FDCAN_TimeoutOccurredCallback");
}

void HAL_FDCAN_HighPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan) {
        BSSR_CAN_Log("HAL_FDCAN_HighPriorityMessageCallback");
}

void HAL_FDCAN_ClockCalibrationCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ClkCalibrationITs){
    BSSR_CAN_Log("HAL_FDCAN_ClockCalibrationCallback");
}

void HAL_FDCAN_TimestampWraparoundCallback(FDCAN_HandleTypeDef *hfdcan) {
    BSSR_CAN_Log("HAL_FDCAN_TimestampWraparoundCallback");
}

void HAL_FDCAN_TT_ScheduleSyncCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TTSchedSyncITs) {
    BSSR_CAN_Log("HAL_FDCAN_TT_ScheduleSyncCallback");
}

void HAL_FDCAN_TT_TimeMarkCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TTTimeMarkITs) {
    BSSR_CAN_Log("HAL_FDCAN_TT_TimeMarkCallback");
}

void HAL_FDCAN_TT_StopWatchCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t SWTime, uint32_t SWCycleCount) {
    BSSR_CAN_Log("HAL_FDCAN_TT_StopWatchCallback");
}

void HAL_FDCAN_TT_GlobalTimeCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TTGlobTimeITs) {
    BSSR_CAN_Log("HAL_FDCAN_TT_GlobalTimeCallback");
}
