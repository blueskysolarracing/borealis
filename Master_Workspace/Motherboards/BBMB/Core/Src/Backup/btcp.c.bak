#include "btcp.h"

#define TCP_TRX_TASK_STACK_SIZE 256
#define TCP_TX_TASK_PRIORITY 4
#define TCP_RX_TASK_PRIORITY 5
#define TCP_TX_QUEUE_SIZE 64
#define BSSR_SERIAL_START 0xa5
#define BSSR_SERIAL_ESCAPE 0x5a

// ########  ##     ##
// ##     ## ##     ##
// ##     ## ##     ##
// ########  ##     ##
// ##         ##   ##
// ##          ## ##
// ##           ###

// ########  ######## ########
// ##     ## ##       ##     ##
// ##     ## ##       ##     ##
// ########  ######   ########
// ##        ##       ##
// ##        ##       ##
// ##        ##       ##

B_tcpHandle_t* B_tcpStart(B_uartHandle_t** transmitBuarts, 
                            B_uartHandle_t* rxBuart,
                            uint8_t numTransmitBuarts,
                            CRC_HandleTypeDef* crc);
//static void tcpTxTask(void *pv);
static void tcpRxTask(void *pv);

// ######## ##     ## ##    ##  ######
// ##       ##     ## ###   ## ##    ##
// ##       ##     ## ####  ## ##
// ######   ##     ## ## ## ## ##
// ##       ##     ## ##  #### ##
// ##       ##     ## ##   ### ##    ##
// ##        #######  ##    ##  ######


B_tcpHandle_t* B_tcpStart(B_uartHandle_t** transmitBuarts, 
                            B_uartHandle_t* rxBuart,
                            uint8_t numTransmitBuarts,
                            CRC_HandleTypeDef* crc){
    B_tcpHandle_t *btcp;
    btcp = pvPortMalloc(sizeof(B_tcpHandle_t));
    btcp->numTransmitBuarts = numTransmitBuarts;
    btcp->transmitBuarts = transmitBuarts;
    btcp->rxBuart = rxBuart;
    btcp->tcpSeqNum = 0;
    btcp->crc = crc;
    btcp->txQ = xQueueCreate(TCP_TX_QUEUE_SIZE, sizeof(B_tcpPacket_t));
    //hpQ = xQueueCreate(10, sizeof(uint8_t));
    //xTaskCreate(tcpTxTask, "tcpTxTask", TCP_TRX_TASK_STACK_SIZE, btcp, TCP_TX_TASK_PRIORITY, &btcp->txTask);
    xTaskCreate(tcpRxTask, "tcpRxTask", TCP_TRX_TASK_STACK_SIZE, btcp, TCP_TX_TASK_PRIORITY, &btcp->rxTask);
    //xTaskCreate(highPowerTask, "highPowerTask", 1024, NULL, 5, NULL);
    return btcp;
}

void B_tcpSend(B_tcpHandle_t *btcp, uint8_t *msg, uint8_t length){
    uint8_t *buf = pvPortMalloc(sizeof(uint8_t)*(MAX_PACKET_SIZE+8));
    buf[0] = BSSR_SERIAL_START;
    buf[1] = length;
    buf[2] = TCP_ID;
    buf[3] = btcp->tcpSeqNum;
    memcpy(buf+4, msg, length);
    uint32_t crc_result = ~HAL_CRC_Calculate(btcp->crc, buf, length+4);
    uint16_t buf_pos = 4;
    if((length + 4) == BSSR_SERIAL_START || (length + 4) == BSSR_SERIAL_ESCAPE){
        buf_pos++;
        buf[3] = length + 4;
        buf[2] = BSSR_SERIAL_ESCAPE;
    }
    if(btcp->tcpSeqNum == BSSR_SERIAL_START || btcp->tcpSeqNum == BSSR_SERIAL_ESCAPE){
        buf[buf_pos -1] = BSSR_SERIAL_ESCAPE;
        buf_pos++;
    }
    buf[buf_pos -1] = btcp->tcpSeqNum;
    btcp->tcpSeqNum++;
    for(int i = 0; i < length; i++){
        if(msg[i] == BSSR_SERIAL_ESCAPE || msg[i] == BSSR_SERIAL_START){
            buf[buf_pos] = BSSR_SERIAL_ESCAPE;
            buf_pos++;
        }
        buf[buf_pos] = msg[i];
        buf_pos++;
    }
    for(int i = 0; i < 4; i++){
        buf[buf_pos] = (crc_result>>(8*(3-i))) &255;
        if(buf[buf_pos] == BSSR_SERIAL_ESCAPE || buf[buf_pos] == BSSR_SERIAL_START){
            buf[buf_pos+1] = buf[buf_pos];
            buf[buf_pos] = BSSR_SERIAL_ESCAPE;
            buf_pos++;
        }
        buf_pos++;
    }
    for(int i = 0; i < btcp->numTransmitBuarts; i++){
        B_uartSend(btcp->transmitBuarts[i], buf, buf_pos);
    }
    vPortFree(buf);
}

//  ######  ########    ###    ######## ####  ######
// ##    ##    ##      ## ##      ##     ##  ##    ##
// ##          ##     ##   ##     ##     ##  ##
//  ######     ##    ##     ##    ##     ##  ##
//       ##    ##    #########    ##     ##  ##
// ##    ##    ##    ##     ##    ##     ##  ##    ##
//  ######     ##    ##     ##    ##    ####  ######

static void tcpRxTask(void *pv){
    B_tcpHandle_t* btcp = pv;
    B_bufQEntry_t *e;
    uint8_t input_buffer[MAX_PACKET_SIZE + 4];
    uint8_t raw_input_buffer[(MAX_PACKET_SIZE + 8)*2]; // Just in case every byte is escaped
    uint8_t escaped = 0;
    uint16_t buf_pos = 0;
    uint16_t raw_buf_pos = 0;
    uint8_t expected_length = 0;
    uint8_t started = 0;
    uint8_t sender = 0;
    uint16_t seqNum = 0xffff;
    uint8_t crcAcc = 0;
    uint32_t crc = 0;
    uint32_t crcExpected = 0;
    B_tcpPacket_t pkt;
    for(;;){
        e = B_uartRead(btcp->rxBuart);
        for(int i = 0; i < e->len; i++){
            raw_input_buffer[raw_buf_pos] = e->buf[i];
            raw_buf_pos++;
            if(e->buf[i] == BSSR_SERIAL_ESCAPE && !escaped){
                escaped = 1;
            } else {
                escaped = 0;
            }
            if(!started){
                if(e->buf[i] == BSSR_SERIAL_START){
                    started = 1;
                    input_buffer[buf_pos] = e->buf[i];
                    buf_pos++;
                }
            } else if(!expected_length){
                expected_length = e->buf[i];
                input_buffer[buf_pos] = e->buf[i];
                buf_pos++;
            } else if(!sender){
                sender = e->buf[i];
                input_buffer[buf_pos] = e->buf[i];
                buf_pos++;
            } else if(seqNum == 0xffff){
                seqNum = e->buf[i];
                input_buffer[buf_pos] = e->buf[i];
                buf_pos++;
            } else if(buf_pos < expected_length+4){
                input_buffer[buf_pos] = e->buf[i];
                buf_pos++;
            } else if(buf_pos + crcAcc < expected_length+8){
                crc |= e->buf[i] << ((3-crcAcc)*8);
                crcAcc++;
                if(crcAcc == 4){
                	crcExpected = ~HAL_CRC_Calculate(btcp->crc, input_buffer, buf_pos);
					if(crcExpected == crc && sender != TCP_ID){
						for(int i = 0; i < btcp->numTransmitBuarts; i++){
							B_uartSend(btcp->transmitBuarts[i], raw_input_buffer, raw_buf_pos);
						}
						pkt.length = expected_length;
						pkt.sender = sender;
						pkt.seqNum = seqNum;
						pkt.payload = input_buffer;
						pkt.crc = crc;
						serialParse(&pkt);
					}
					raw_buf_pos = 0;
					crc = 0;
					seqNum = 0xffff;
					crcAcc = 0;
					crcExpected = 0;
					sender = 0;
					buf_pos = 0;
					expected_length = 0;
					started = 0;
                }
            }
        }
        B_uartDoneRead(e);
    }
}

__weak void serialParse(B_tcpPacket_t *pkt){
//	switch(pkt->sender){
//	case 0x04:
//		  if(pkt->payload[4] == 0x01){
//			  xQueueSend(hpQ, pkt->payload+5, 0);
//		  } else if(pkt->payload[4] == 0x04){
//			  if(pkt->payload[5]){
//				  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);
//			  } else {
//				  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);
//			  }
//		  }
//	}
}
