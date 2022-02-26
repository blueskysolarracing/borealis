#include "btcp.h"
//#define PAD
// Define pad if we need to pad buffer to multiple of 4

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


//static void tcpTxTask(void *pv);
static void tcpRxTask(void *pv);

// ######## ##     ## ##    ##  ######
// ##       ##     ## ###   ## ##    ##
// ##       ##     ## ####  ## ##
// ######   ##     ## ## ## ## ##
// ##       ##     ## ##  #### ##
// ##       ##     ## ##   ### ##    ##
// ##        #######  ##    ##  ######

/** B_tcpStart
  * @brief  Function which creates a rxTask that will be constantly looking for received data.
			Any relevant info will be stored in B_tcpHandle_t, which is allocated in this function and then returned.
  * @param  senderID: the ID of your motherboard. See protocol.h for your board's corresponding ID. 
			transmitBuarts : a pointer to a pointer to B_uartHandle_t, or equivalently, an array of B_uartHandle_t* used to transmit messages. 
				This parameter can also be written as: B_uartHandle_t* transmitBuarts[]
				The reason transmitBuarts is an array is to be able to transmit the same message over multiple uart ports when calling B_tcpSend()
			rxBuart : a pointer to B_uartHandle_t used to receive messages. 
			numTransmitBuarts: the size of the transmitBuarts array. In other words, this is the number of uart ports over which we are sending the same messages. 
			crc: Pointer to hcrc - the crc handle. Understand CRC: https://www.youtube.com/watch?v=1WAtFzkfpLI
  * @note	Call this function in the setup section in main(). 
  * @retval B_tcpHandle_t*: pointer to a B_tcpHandle_t struct which stores uart, task handles and other transmission information 
  */
B_tcpHandle_t* B_tcpStart(uint8_t senderID, B_uartHandle_t** transmitBuarts,
                            B_uartHandle_t* rxBuart,
                            uint8_t numTransmitBuarts,
                            CRC_HandleTypeDef* crc){
    B_tcpHandle_t *btcp;
    btcp = pvPortMalloc(sizeof(B_tcpHandle_t));
    btcp->numTransmitBuarts = numTransmitBuarts;
    btcp->transmitBuarts = pvPortMalloc(sizeof(B_tcpHandle_t*)*numTransmitBuarts);
    for(int i = 0; i < numTransmitBuarts; i++){
    	btcp->transmitBuarts[i] = transmitBuarts[i];
    }
    btcp->senderID = senderID;
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

/** B_tcpSend
  * @brief  Function which enqueues the input messages into FreeRTOS queues, which behave in FIFO order. 
			The queues are constantly read by the txTasks in buart.c, which dequeue the messages. 
			The messages that are dequeued are then sent out over the corresponding UART ports stored in the btcp->transmitBuarts array. 
  * @param  btcp : a pointer to B_tcpHandle_t
			msg: an array of messages you want to transmit
			length: the length of the array of messages
			senderAddress: the address of the sender
  * @note	msg array can be up to MAX_PACKET_SIZE (256) bytes long if it does not contain values that must be escaped. 
			If it contains values that need to be escaped, the msg array can be up to (MAX_PACKET_SIZE - number_of_values_to_be_escaped) bytes long
  * @retval B_tcpHandle_t*: pointer to a B_tcpHandle_t struct which stores uart, task handles and other transmission information 
  */
void B_tcpSend(B_tcpHandle_t *btcp, uint8_t *msg, uint8_t length){
	
    uint8_t *buf = pvPortMalloc(sizeof(uint8_t)*(MAX_PACKET_SIZE+8)); 

    //buf without escape character to generate crc
	buf[0] = BSSR_SERIAL_START;
    buf[1] = length;
<<<<<<< HEAD
    buf[2] = TCP_ID;
=======
    buf[2] = btcp->senderID;
>>>>>>> 03d66fc9dd61414be8e6f154f7d034ab98ed6c93
    buf[3] = btcp->tcpSeqNum;
    memcpy(buf+4, msg, length);
    uint32_t crc_result = ~HAL_CRC_Calculate(btcp->crc, (uint32_t*)buf, length+4);

    //buf with escape character
    uint16_t buf_pos = 0;
	
    buf[buf_pos] = BSSR_SERIAL_START;
    buf_pos++;
	
    //check if length needs to be escaped
	if(length == BSSR_SERIAL_START || length == BSSR_SERIAL_ESCAPE){
    	buf[buf_pos] = BSSR_SERIAL_ESCAPE;
    	buf_pos++;
    	buf[buf_pos] = length;
    	buf_pos++;
    } else{
    	buf[buf_pos] = length;
    	buf_pos++;
    }
	
<<<<<<< HEAD
    buf[buf_pos] = TCP_ID;
=======
    buf[buf_pos] = btcp->senderID;;
>>>>>>> 03d66fc9dd61414be8e6f154f7d034ab98ed6c93
    buf_pos++;

    //check if sequence number needs to be escaped
    if(btcp->tcpSeqNum == BSSR_SERIAL_START || btcp->tcpSeqNum == BSSR_SERIAL_ESCAPE){
    	buf[buf_pos] = BSSR_SERIAL_ESCAPE;
    	buf_pos++;
    	buf[buf_pos] = btcp->tcpSeqNum;
    	buf_pos++;
    } else{
    	buf[buf_pos] = btcp->tcpSeqNum;
    	buf_pos++;
    }
    btcp->tcpSeqNum++;

    //check if each data needs to be escaped
    for(int i=0; i<length; i++){
    	if(msg[i] == BSSR_SERIAL_ESCAPE || msg[i] == BSSR_SERIAL_START){
    		buf[buf_pos] = BSSR_SERIAL_ESCAPE;
    		buf_pos++;
    		buf[buf_pos] = msg[i];
    		buf_pos++;
    	} else{
    		buf[buf_pos] = msg[i];
    		buf_pos++;
    	}
    }

	//checks if each crc value needs to be escaped
    for(int i=0; i<4; i++){
    	uint8_t crc = (crc_result>>(8*(3-i))) & 255;
    	if(crc == BSSR_SERIAL_ESCAPE || crc == BSSR_SERIAL_START){
    		buf[buf_pos] = BSSR_SERIAL_ESCAPE;
    		buf_pos++;
    		buf[buf_pos] = crc;
    		buf_pos++;
    	} else{
    		buf[buf_pos] = crc;
    		buf_pos++;
    	}
    }
#ifdef PAD
    if(buf_pos%4 != 0) {
    	int paddingNum = 4 - buf_pos % 4;
    	for (int i = paddingNum; i > 0; i--) {
    	   buf[buf_pos] = 0x00;
    	   buf_pos++;
    	}
   }
#endif
	// Send the message to the Queue corresponding to each of the UART ports in the transmitBuarts array 
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

			// First, check if there is an escape character and act accordingly
            if(e->buf[i] == BSSR_SERIAL_ESCAPE && !escaped){ 
                escaped = 1;
				continue; //Go to the next loop iteration
            } else if (escaped) {
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
                	//crcExpected = ~HAL_CRC_Calculate(btcp->crc, input_buffer, buf_pos);
                	//GEN11 change:
                	crcExpected = ~HAL_CRC_Calculate(btcp->crc, (uint32_t*)input_buffer, buf_pos);

					if(crcExpected == crc && sender != TCP_ID){ // If CRC correct and the sender is not this motherboard
						/*for(int i = 0; i < btcp->numTransmitBuarts; i++){
							B_uartSend(btcp->transmitBuarts[i], raw_input_buffer, raw_buf_pos);
						}*/  //Commented out since this is for Daisy Chain, and we are not doing Daisy Chain this cycle
						pkt.length = expected_length;
						pkt.sender = sender;
						pkt.senderID = sender;
						pkt.seqNum = seqNum;
						pkt.payload = input_buffer;
						pkt.data = pkt.payload + 4; //points to element containing DataID
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
