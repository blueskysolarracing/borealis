/********************************************************************************************************
USER LICENCE AGREEMENT and DISCLAIMER:
This source code is the property of NanoGrid Ltd.
Any modification made to this source code remains the property of NanoGrid Ltd.
The source code may only be used for specific authorized peak powers.
The source code is confidential and must be protected and must not be distributed.
NanoGrid Ltd assumes no liability for damage or harm or losses caused by this source code.
Deviation from this agreement requires permission from NanoGrid Ltd.
This agreement must appear at the top of each source file and applies to the source as well
 as accompanying binary, object, library, or any related files.
Cantact: Tom Rodinger, tom@nanoleaf.me
********************************************************************************************************/

#include <p24hj64gp502.h>
#include "types.h"

#define PPT_NODE_ID 0 // leave this tag as 0, the actual tag gets filled in in software
#define FULL_ID   (((uint32)0x18FF << 16) | ((uint32)PPT_NODE_ID << 12) | (uint32)0xF5)
#define SID_11B   (FULL_ID >> 18)
#define EID_12MSB ((FULL_ID >> 6) & 0b111111111111)
#define EID_6LSB  (FULL_ID & 0b111111)
#define SRR 0b0
#define IDE 0b1
#define MESSAGE_BUFFER_WORD0 ((SID_11B << 2) | (SRR << 1) | IDE)
#define MESSAGE_BUFFER_WORD1 (EID_12MSB)
#define RTR 0b0
#define RB1 0b0
#define RB0 0b0
#define DLC 0b1000
#define MESSAGE_BUFFER_WORD2 ((EID_6LSB << 10) | (RTR << 9) | (RB1 << 8) | (RB0 << 4) | DLC)

float payloadf;

/* Assign 4 x 8 word Message Buffers for ECAN1 in DMA RAM */
uint16 CAN_message_buff[4][8] __attribute__((space(dma)));
uint32 message_buffer_word1;

void set_ppt_node_id(uint16 ppt_node_id)
{
    message_buffer_word1 = MESSAGE_BUFFER_WORD1 | ((ppt_node_id & 0b1111) << 6);
	C1CTRL1bits.WIN = 0x1;
	C1RXF0EIDbits.EID = 0x00B0 | (ppt_node_id << 12);
	C1CTRL1bits.WIN = 0x0;
}

void CAN_init(uint16 ppt_node_id)
{
    // put the module in configuration mode
    C1CTRL1bits.REQOP=4;
    while(C1CTRL1bits.OPMODE != 4);

    // Set up bit timing for a bit rate of 250kbps
    // Fcy = 16MHz
    // Ftq = 4MHz
    // N = 16
    // Bit Time = N * Tq = (Sync Segment + Propagation Delay + Phase Segment 1 + Phase Segment 2)
    // Tq = 1 / Ftq
    // Phase Segment 1 = 4 Tq
    // Phase Segment 2 = 4 Tq
    // Propagation Delay = 7 Tq
    // Sync Segment = 1 Tq
    C1CFG1bits.BRP = 1;        // CiCFG1<BRP> = (Fcy / (2 * Ftq) - 1 = 1
    C1CFG1bits.SJW = 0x3;      // Synchronization Jump Width set to 4 TQ
    C1CFG2bits.SEG1PH = 0x3;   // Phase Segment 1 time is 4 TQ
    C1CFG2bits.SEG2PHTS = 0x1; // Phase Segment 2 time is set to be programmable
    C1CFG2bits.SEG2PH = 0x3;   // Phase Segment 2 time is 4 TQ
    C1CFG2bits.PRSEG = 0x6;    // Propagation Segment time is 7 TQ
    C1CFG2bits.SAM = 0x1;      // Bus line is sampled three times at the sample point
    C1FCTRLbits.DMABS=0b000;   // 4 CAN Messages to be buffered in DMA RAM

	/* Enable window to access acceptance filter registers */
	C1CTRL1bits.WIN = 0x1;

	/* Configure Mask register 0 to mask only the command
	Mask Bits (29-bits) : 0b1 1111 1111 1111 1111 0000 1111 1111
	SID<10:0> : 0b11111111111 ..SID<10:0> or EID<28:18>
	EID<17:16> : 0b11 ..EID<17:16>
	EID<15:0> : 0b1111000011111111 ..EID<15:0> */
	C1RXM0SIDbits.SID = 0b11111111111;
	C1RXM0SIDbits.EID = 0b11;
	C1RXM0EIDbits.EID = 0b1111000011111111;

	/* Create a filter to receive PPT node tag specific messages, filter 0, mask 0, message buffer 2 */
	/* Select Mask for Acceptance Filter */
	C1FMSKSEL1bits.F0MSK=0b00;
	/* Configure Filter to match extended identifier 
	Filter Bits (29-bits) : 18F5yxB0  , where y is the node ID and x is the don't bits (the command) 
	SID<10:0> : 1 1000 1111 01 ..SID<10:0> or EID<28:18> 
	EID<17:16> : 01 ..EID<17:16>
	EID<15:0> : yyyy xxxx 1011 0000 ..EID<15:0> */
	C1RXF0SIDbits.SID = 0b11000111101; 
	C1RXF0SIDbits.EID = 0b01;
	C1RXF0EIDbits.EID = 0b1111000010110000; // make the ppt node ID 0xF for now, will get set to the right value shortly
	/* Filter to check for Extended Identifier */
	C1RXM0SIDbits.MIDE = 0x1;
	C1RXF0SIDbits.EXIDE= 0x1;
	/* Select message buffer to use for received messages */
	C1BUFPNT1bits.F0BP = 0x2;
	/* Enable Filter */
	C1FEN1bits.FLTEN0=0x1;

	/* Create a filter to receive PPT broadcast messages, filter 1, mask 0, messge buffer 3 */
	/* Select Mask for Acceptance Filter */
	C1FMSKSEL1bits.F1MSK=0b00;
	/* Configure Filter to match extended identifier
	Filter Bits (29-bits) : 0x18F5FxB0  , where y is the node ID and x is the don't bits (the command) 
                            0b1 1000 1111 0101 1111 xxxx 1011 0000
	SID<10:0> : 1 1000 1111 01 ..SID<10:0> or EID<28:18> 
	EID<17:16> : 01 ..EID<17:16>
	EID<15:0> : 1111 xxxx 1011 0000 ..EID<15:0> */
	C1RXF1SIDbits.SID = 0b11000111101; 
	C1RXF1SIDbits.EID = 0b01;
	C1RXF1EIDbits.EID = 0b1111000010110000;
	/* Filter to check for Extended Identifier */
	C1RXM1SIDbits.MIDE = 0x1;
	C1RXF1SIDbits.EXIDE= 0x1;
	/* Select message buffer to use for received messages */
	C1BUFPNT1bits.F1BP = 0x3;
	/* Enable Filter */
	C1FEN1bits.FLTEN1=0x1;

	C1CTRL1bits.WIN = 0x0;

    // put the module in normal mode
    C1CTRL1bits.REQOP=0;
    while(C1CTRL1bits.OPMODE != 0);

    // clear the buffer and overflow flags
    C1RXFUL1 = 0;
    C1RXFUL2 = 0;
    C1RXOVF1 = 0;
    C1RXOVF2 = 0;
    C1TR01CONbits.TXEN0=1;     // ECAN1, buffer 0 is a transmit buffer
    C1TR01CONbits.TX0PRI=0b01; // set transmit priority level to 1 (second lowest)
    C1TR01CONbits.TXEN1=1;     // ECAN1, buffer 1 is a transmit buffer
    C1TR01CONbits.TX1PRI=0b01; // set transmit priority level to 1 (second lowest)
    C1TR23CONbits.TXEN2=0;     // ECAN1, buffer 2 is a receive buffer
    C1TR23CONbits.TXEN3=0;     // ECAN1, buffer 3 is a receive buffer

    // Initialize the DMA channel 1 for ECAN TX
    DMACS0 = 0;               // clear the colission flags
    DMA1CONbits.SIZE = 0x0;   // Data Transfer Size: Word Transfer Mode
    DMA1CONbits.DIR = 0x1;    // Data Transfer Direction: DMA RAM to Peripheral
    DMA1CONbits.AMODE = 0x2;  // DMA Addressing Mode: Peripheral Indirect Addressing mode
    DMA1CONbits.MODE = 0x0;   // Operating Mode: Continuous, Ping-Pong modes disabled
    DMA1REQ = 0x46;           // Assign ECAN1 Transmit event for DMA Channel 1
    DMA1CNT = 7;              // Set Number of DMA Transfer per ECAN message to 8 words
    DMA1PAD = (uint16)&C1TXD; // Peripheral Address: ECAN1 Transmit Register
    DMA1STA = __builtin_dmaoffset(CAN_message_buff); // Start Address Offset for ECAN1 Message Buffer 0
    DMA1CONbits.CHEN = 0x1;   // Channel Enable: Enable the DMA Channel */

    // Initialize the DMA channel 2 for ECAN RX
    DMACS0 = 0;               // clear the colission flags
    DMA2CONbits.SIZE = 0x0;   // Data Transfer Size: Word Transfer Mode
    DMA2CONbits.DIR = 0x0;    // Data Transfer Direction: Peripheral to DMA RAM
    DMA2CONbits.AMODE = 0x2;  // DMA Addressing Mode: Peripheral Indirect Addressing mode
    DMA2CONbits.MODE = 0x0;   // Operating Mode: Continuous, Ping-Pong modes disabled
    DMA2REQ = 0x22;           // Assign ECAN1 Receive event for DMA Channel 2
    DMA2CNT = 7;              // Set Number of DMA Transfer per ECAN message to 8 words
    DMA2PAD = (uint16)&C1RXD; // Peripheral Address: ECAN1 Transmit Register
    DMA2STA = __builtin_dmaoffset(CAN_message_buff); // Start Address Offset for ECAN1 Message Buffer 0
    DMA2CONbits.CHEN = 0x1;   // Channel Enable: Enable the DMA Channel */

    RPINR26bits.C1RXR = 10;    // RP10 input pin assigned to CAN receive function
    RPOR5bits.RP11R = 0b10000; // CAN transmit function (0b10000) is assigned to pin RP11

	set_ppt_node_id(ppt_node_id);
}

void CAN_send(uint16 Vpaneli, uint16 Ppaneli, uint16 Vbatteryi, uint16 Ipaneli)
{
    float Vpanelf =   (float)Vpaneli / 10;
    float Ppanelf =   (float)Ppaneli / 10;
    float Vbatteryf = (float)Vbatteryi / 10;
    float Ipanelf =   (float)Ipaneli / 1000;

    // we will transmit using two message buffers

    if((C1TR01CONbits.TXREQ0 == 1) || (C1TR01CONbits.TXREQ1 == 1)) {
        return;  // in case still transmitting, don't mess with the buffers, just discard this data
    }

    DMACS0 = 0; // clear the colission flags

    // fill the first message buffer
    CAN_message_buff[0][0] = MESSAGE_BUFFER_WORD0;
    CAN_message_buff[0][1] = message_buffer_word1 | 0b000;
    CAN_message_buff[0][2] = MESSAGE_BUFFER_WORD2;

    // write 4 bytes message
    CAN_message_buff[0][3] = ((uint16*)&Vpanelf)[0];
    CAN_message_buff[0][4] = ((uint16*)&Vpanelf)[1];
    CAN_message_buff[0][5] = ((uint16*)&Ppanelf)[0];
    CAN_message_buff[0][6] = ((uint16*)&Ppanelf)[1];
    CAN_message_buff[0][7] = 0;

    C1TR01CONbits.TXREQ0 = 0x1; // start transmission of first message buffer

    CAN_message_buff[1][0] = MESSAGE_BUFFER_WORD0;
    CAN_message_buff[1][1] = message_buffer_word1 | 0b100;
    CAN_message_buff[1][2] = MESSAGE_BUFFER_WORD2;

    // write 4 bytes message
    CAN_message_buff[1][3] = ((uint16*)&Vbatteryf)[0];
    CAN_message_buff[1][4] = ((uint16*)&Vbatteryf)[1];
    CAN_message_buff[1][5] = ((uint16*)&Ipanelf)[0];
    CAN_message_buff[1][6] = ((uint16*)&Ipanelf)[1];
    CAN_message_buff[1][7] = 0;

    C1TR01CONbits.TXREQ1 = 0x1; // start transmission of second message buffer
}

void CAN_send_status(uint16 status1)
{
    if(C1TR01CONbits.TXREQ0 == 1) {
        return;  // in case still transmitting, don't mess with the buffers, just discard this data
    }

    DMACS0 = 0; // clear the colission flags

    // fill the message buffer
    CAN_message_buff[0][0] = MESSAGE_BUFFER_WORD0;
    CAN_message_buff[0][1] = message_buffer_word1 | 0b1000;
    CAN_message_buff[0][2] = MESSAGE_BUFFER_WORD2;

    // write 4 bytes message
    CAN_message_buff[0][3] = status1;
    CAN_message_buff[0][4] = 0;
    CAN_message_buff[0][5] = 0;
    CAN_message_buff[0][6] = 0;
    CAN_message_buff[0][7] = 0;

    C1TR01CONbits.TXREQ0 = 0x1; // start transmission of first message buffer
}

uint16 CAN_receive_command(void)
{
	int16 command = -1;

	if(C1RXFUL1bits.RXFUL2) {
		C1RXFUL1bits.RXFUL2 = 0;
		C1RXOVF1bits.RXOVF2 = 0;
		command = (CAN_message_buff[2][1] >> 2) & 0xF;
		if( (command == 2) || (command == 3) ) {
		    payloadf = *((float *)&CAN_message_buff[2][3]);
		}
	}

	if(C1RXFUL1bits.RXFUL3) {
		C1RXFUL1bits.RXFUL3 = 0;
		C1RXOVF1bits.RXOVF3 = 0;
		command = (CAN_message_buff[3][1] >> 2) & 0xF;
		if( (command == 2) || (command == 3) ) {
		    payloadf = *((float *)&CAN_message_buff[3][3]);
		}
	}


	return command;
}

uint16 CAN_get_payload_voltage(void)
{
    return (uint16)(payloadf * 10.0);
}

uint16 CAN_get_payload_voltage_ADC(void)
{
    return (uint16)(payloadf * 20.0);
}
