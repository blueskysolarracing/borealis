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

uint8 char_count;
uint16 send_value;
uint8 decimal_location;
uint8 print_zeros;

#define BUFFER_SIZE 8
uint8 buffer[BUFFER_SIZE];
uint16 buffer_ptr;
uint8 bytes_to_transmit;
uint8 transmitting;


void UART_init(void)
{
//	U2BRG = 424; // for 9600 baud
	U2BRG = 208; // for 19200 baud
	U2MODE = 0b1000000000001000;
	U2STAbits.UTXEN = 1;
	RPINR19bits.U2RXR = 6;  // Assign RP6 pin to receive function of USART
	RPOR2bits.RP5R = 5;     // Assign RP5 pin to transmit function of UART

	char_count = 0;

	buffer_ptr = 0;
	transmitting = 0;
}

void UART_receive(void)
{
	uint16 received_byte; 

	U2STAbits.OERR = 0; // clear any overrun error
	if(U2STAbits.URXDA) {
		received_byte = U2RXREG;
		U2TXREG = received_byte;
	}
}

const uint16 conversion_multiplier[5] = {1, 10, 100, 1000, 10000};

void UART_send_number(uint16 num, uint8 dec_loc) 
{
	if(char_count != 0) { // if a number is already being sent, silently drop this new request
		return;
	}
	send_value = num;
	if(dec_loc == 0) {
		dec_loc = 127;
	}
	decimal_location = dec_loc + 2;
	char_count = 7;
	print_zeros = 0;
}

void UART_send_divider(void)
{
	if(char_count != 0) { // if a number is already being sent, silently drop this new request
		return;
	}
	U2TXREG = '-';  // the divider is just a minus sign
	decimal_location = 127 + 2;
	print_zeros = 1;
	char_count = 2;
}

void UART_send(void)
{
	uint16 cm;
	uint16 n;

	if(U2STAbits.UTXBF == 0) {
		if(char_count == 0) {
			return;
		}

		if(char_count == decimal_location) {
			if(!print_zeros) {
				U2TXREG = '0';
				print_zeros = 1;
			}
			else {
				U2TXREG = '.';
				decimal_location = -1;
			}
			return;
		}

		switch(char_count) {
		case 1:
			U2TXREG = '\n';
			break;
		case 2:
			if(!print_zeros) {
				U2TXREG = '0';
				print_zeros = 1;
				return;
			}
			U2TXREG = '\r';
			break;
		default:
			cm = conversion_multiplier[char_count - 3];
			n = send_value / cm;
			if(n > 0) { 
				U2TXREG = n + '0';
				print_zeros = 1;
			}
			else if(print_zeros) {
				U2TXREG = '0';
			}
			send_value = send_value - n * cm;
		}
		char_count--;
	}	
}

void UART_queue_binary(uint32 value, uint8 length)
{
	uint16 i;
	
	if(transmitting) {
		return;
	}

	for(i = 0; i< length; i++)
	{
		if(buffer_ptr < BUFFER_SIZE) {
			buffer[buffer_ptr++] = ((uint8*)&value)[i];
		}
	}
}

void UART_start_transmission(void)
{
	transmitting = 1;
	bytes_to_transmit = buffer_ptr;
	buffer_ptr = 0;
}

void UART_process(void)
{
	if(transmitting) {

		while(U2STAbits.UTXBF == 0) {
			if(buffer_ptr == bytes_to_transmit) {
				transmitting = 0;
				buffer_ptr = 0;
				return;
			}

			U2TXREG = buffer[buffer_ptr++];
		}
	}
}


