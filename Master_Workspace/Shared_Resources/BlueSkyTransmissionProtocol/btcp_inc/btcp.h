#ifndef __BTCP_H
#define __BTCP_H


#include "buart.h"
#include "main.h"
#include "protocol_ids.h"
#include "cmsis_os.h"
#include "string.h"
#define MAX_PACKET_SIZE 256
#define TCP_TRX_TASK_STACK_SIZE 256
#define TCP_TX_TASK_PRIORITY 4
#define TCP_RX_TASK_PRIORITY 5
#define TCP_TX_QUEUE_SIZE 64
#define BSSR_SERIAL_START 0xa5
#define BSSR_SERIAL_ESCAPE 0x5a


typedef struct {
	uint8_t sender;
	uint8_t senderID;
	// Note sender and senderID are equivalent
	// senderID is added to make the naming more clear
	// sender is still kept for legacy code support

	uint8_t length;
    uint8_t seqNum;
    uint8_t *payload; // Header + CRC
    uint8_t *data;  // will point to payload+4
    uint32_t crc;
} B_tcpPacket_t;

typedef struct {
	uint8_t senderID;
    B_uartHandle_t**    transmitBuarts;
    uint8_t             numTransmitBuarts;
    B_uartHandle_t*     rxBuart;
    uint8_t             tcpSeqNum;
    QueueHandle_t       txQ;
    TaskHandle_t        rxTask;
    CRC_HandleTypeDef*  crc;
} B_tcpHandle_t;


B_tcpHandle_t* B_tcpStart(uint8_t sender, B_uartHandle_t** transmitBuarts,
                            B_uartHandle_t* rxBuart, 
                            uint8_t numTransmitBuarts,
                            CRC_HandleTypeDef* crc);
void serialParse(B_tcpPacket_t *pkt);  

void B_tcpSend(B_tcpHandle_t *btcp, uint8_t *msg, uint8_t length);

#endif
