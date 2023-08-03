/*
 * uMPPT.c
 *
 *  Created on: Jul 22, 2023
 *      Author: tonyh
 */

#include "uMPPT.h"


void config_uMPPT(struct board_param* board){

	for(int i = 0; i < NUM_UMPPT; i++){
		board->uMPPTs[i]->uMPPT_ID = i;
		board->uMPPTs[i]->pwm_num = i;
		board->uMPPTs[i]->pwm_duty_cycle = DEFAULT_PWM_DUTY_CYCLE;
		board->uMPPTs[i]->pwm_frequency = DEFAULT_PWM_FREQ;
		board->uMPPTs[i]->prev_input_voltage = 0; 
		board->uMPPTs[i]->MPP_voltage = 0;
		board->uMPPTs[i]->MPP_current = 0;
	}

	// add configs to the adc on board the stm32

}

float SPI_read_ADC(uint8_t uMPPT_ID, struct board_param* board){

	uint8_t voltage = {0, 0};

	HAL_GPIO_WritePin(board->CS_Ports[uMPPT_ID], board->CS_Pins[uMPPT_ID], GPIO_PIN_RESET);
	
	if(HAL_SPI_Receive(board->hspi_handle, voltage, 1, 100) == HAL_OK){

		if(DEBUG_MODE){
			char print_buf[100];
			sprintf(print_buf, "ADC reading for CH %d: 0x%x%x\n", uMPPT_ID, voltage[0], voltage[1]);
			HAL_UART_Transmit(board->huart_handle, (uint8_t*) print_buf, strlen(print_buf), 10);
		}
	}

	HAL_GPIO_WritePin(board->CS_Ports[uMPPT_ID], board->CS_Pins[uMPPT_ID], GPIO_PIN_SET);
	uint16_t raw_voltage = (voltage[0] << 8) | (voltage[1]);

	return (float) (raw_voltage / ADC_CONV_FACTOR);

}

float STM_read_ADC(struct board_param* board){

}

