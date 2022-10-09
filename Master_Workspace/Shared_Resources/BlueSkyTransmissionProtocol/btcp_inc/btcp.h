#ifndef __BTCP_H
#define __BTCP_H

#include "buart.h"
#include "main.h"
#include "protocol_ids.h"
#include "cmsis_os.h"
#include "string.h"
#include "pack_data.h"
#define MAX_PACKET_SIZE 256
#define TCP_TRX_TASK_STACK_SIZE 256
#define TCP_TX_TASK_PRIORITY 4
#define TCP_RX_TASK_PRIORITY 5
#define TCP_TX_QUEUE_SIZE 64
#define BSSR_SERIAL_START 0xa5
#define BSSR_SERIAL_ESCAPE 0x5a

#define HEARTBEAT_INTERVAL 1000 //Delay between sending heartbeats

typedef struct B_tcpHandle_t{
	uint8_t senderID;
    B_uartHandle_t**    transmitBuarts;
    uint8_t             numTransmitBuarts;
    B_uartHandle_t*     rxBuart;
    uint8_t             tcpSeqNum;
    QueueHandle_t       txQ;
    TaskHandle_t        rxTask;
    CRC_HandleTypeDef*  crc;
    uint8_t 			txBuf[MAX_PACKET_SIZE];
} B_tcpHandle_t;



typedef struct B_tcpPacket_t{
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
    B_tcpHandle_t * btcp;
    uint16_t raw_buf_length;
    uint8_t* raw_input_buffer; // pointer to raw_input (with escape characters, etc)

} B_tcpPacket_t;



B_tcpHandle_t* B_tcpStart(uint8_t sender, B_uartHandle_t** transmitBuarts,
                            B_uartHandle_t* rxBuart, 
                            uint8_t numTransmitBuarts,
                            CRC_HandleTypeDef* crc);
void serialParse(B_tcpPacket_t *pkt);  



void B_tcpSend(B_tcpHandle_t *btcp, uint8_t *msg, uint8_t length);
void B_tcpSendToBMS(B_tcpHandle_t *btcp, uint8_t *msg, uint8_t length);
void B_tcpSendBlocking(B_tcpHandle_t *btcp, uint8_t *msg, uint8_t length);

#endif
