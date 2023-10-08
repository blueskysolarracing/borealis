/*
 * uMPPT.c
 *
 *  Created on: Jul 22, 2023
 *      Author: tonyh
 */
#include "uMPPT.h"

extern uint8_t uMPPT_SPI_inProgress_ID = -1;


void config_uMPPT(struct board_param* board){

	for(int i = 0; i < NUM_UMPPT; i++){
		board->uMPPTs[i]->uMPPT_ID = i;
		board->uMPPTs[i]->pwm_num = i;
		board->uMPPTs[i]->pwm_duty_cycle = DEFAULT_PWM_DUTY_CYCLE;
		board->uMPPTs[i]->pwm_frequency = DEFAULT_PWM_FREQ;
		board->uMPPTs[i]->prev_input_voltage = 0; 
		board->uMPPTs[i]->MPP_voltage = 0;
		board->uMPPTs[i]->MPP_current = 0;
		board->uMPPTs[i]->sleeping = 0;

	}
	// add configs to the adc on board the stm32
	// config each adc to go to sleep mode

	// Put all ADCs to sleep mode and pull up all CS Pins
	for(int i = 0; i < NUM_UMPPT; i++){
		ADC_sleepMode(i, board);
	}
	//Start PWM
	for (int i = 0; i < NUM_UMPPT; i++){
		HAL_TIM_PWM_Start(board->PWM_TIM[i], board->PWM_CHANNEL[i]);
	}
}

void ADC_sleepMode(uint8_t uMPPT_ID, struct board_param *board){

	HAL_GPIO_WritePin(board->CS_Ports[uMPPT_ID], board->CS_Pins[uMPPT_ID], GPIO_PIN_RESET);
	uint8_t garbage[2] = {0, 0};

	if(HAL_SPI_Receive_IT(board->hspi_handle, garbage, 2)){
		uMPPT_SPI_inProgressID = uMPPT_ID;
	}

	HAL_GPIO_WritePin(board->CS_Ports[uMPPT_ID], board->CS_Pins[uMPPT_ID], GPIO_PIN_SET);

}

float ADC_readVoltage(uint8_t uMPPT_ID, struct board_param* board){

	uint8_t voltage = {0, 0};

	// Read from first channel on STM ADC
	if(uMPPT_ID == 0){

	}

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

float readOutputCurrent(){

}

float readOutputVoltage(){

}

void update_MPP_HillClimb(struct board_param *board, struct uMPPT *target_uMPPT){
	uint8_t found_MPP = 0;

	target_uMPPT->MPPT_in_progress = 1;

	do {
		float raw_ADC = 0.0;
		float prev_power = 0;
		float current_power = 0;

		for(int i = 0; i < NUM_AVG_CURRENT; i++){
			raw_ADC += readOutputCurrent();
		}
		raw_ADC /= NUM_AVG_CURRENT;

		board->i_Out = (((raw_ADC / 4095.0) * VDDA) - I_MEAS_OFFSET) / I_SENSE_AMP_RATIO / I_SHUNT_VALUE;
		target_uMPPT->calc_input_current = board->i_Out * target_uMPPT->pwm_duty_cycle;

		raw_ADC = ADC_readVoltage(target_uMPPT->uMPPT_ID, board);
		target_uMPPT->prev_input_voltage = target_uMPPT->input_voltage;
		target_uMPPT_prev_input_voltage = raw_ADC;


		// Hill Climbing Algorithm
		prev_power = target_uMPPT->prev_input_voltage * target_uMPPT->prev_calc_input_current;
		current_power = target_uMPPT->input_voltage * target_uMPPT->calc_input_current;

		if ( current_power > prev_power + ALLOWED_HillClimb_dP_error ){ //If we are in the direction of increasing power wrt current, increase current

			if (target_uMPPT->input_voltage < MIN_INPUT_VOLTAGE){ //Increasing current will decrease input voltage so we can't go lower if we're already at the lowest
				updatePWMDutyCycle(target_uMPPT, board, DEFAULT_PWM_DUTY_CYCLE); //Go back to default current
			} else {
				updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle - PWM_DUTY_CYCLE_STEP_CHANGE);
			}
		} else if ( current_power < prev_power - ALLOWED_HillClimb_dP_error ){ //If we are in the direction of decreasing power wrt current, decrease current
			updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle + PWM_DUTY_CYCLE_STEP_CHANGE);
		} else { //We are at MPP
			found_MPP = 1;
			target_uMPPT->MPP_voltage = target_uMPPT->input_voltage;
			target_uMPPT->MPP_current = target_uMPPT->calc_input_current;
			if (DEBUG_FLAG){	HAL_UART_Transmit(board->huart_handle, (uint8_t *) "Found MPP!\n", 12, 10);	}
		}

		//Update previous measurements
		target_uMPPT->prev_input_voltage = target_uMPPT->input_voltage;
		target_uMPPT->prev_calc_input_current = target_uMPPT->calc_input_current;

		HAL_Delay(MPPT_ITERATION_DELAY);
	} while (found_MPP != 1);

	target_uMPPT->MPPT_in_progress = 0; //Update flag to tell MPPT is not in progress

}

void updatePWMDutyCycle(struct board_param * board, struct uMPPT *target_uMPPT, float new_duty_cycle){
	//Bound PWM duty cycle
	if (new_duty_cycle < MIN_PWM_DUTY_CYCLE){ new_duty_cycle = MIN_PWM_DUTY_CYCLE; }
	else if (new_duty_cycle > MAX_PWM_DUTY_CYCLE){ new_duty_cycle = MAX_PWM_DUTY_CYCLE; }

	//Update appropriate timer register
	if (target_uMPPT->uMPPT_ID <= 1){ //If uMPPT 1 or 2
		board->PWM_timers[target_uMPPT->uMPPT_ID]->Instance->CCR1 = round(new_duty_cycle * board->PWM_timers[target_uMPPT->uMPPT_ID]->Instance->ARR);
	} else if (target_uMPPT->uMPPT_ID >= 3){ //If uMPPT 4 or 5
		board->PWM_timers[target_uMPPT->uMPPT_ID]->Instance->CCR3 = round(new_duty_cycle * board->PWM_timers[target_uMPPT->uMPPT_ID]->Instance->ARR);
	} else { //If uMPPT #2
		board->PWM_timers[target_uMPPT->uMPPT_ID]->Instance->CCR4 = round(new_duty_cycle * board->PWM_timers[target_uMPPT->uMPPT_ID]->Instance->ARR);
	}
	target_uMPPT->pwm_duty_cycle = new_duty_cycle;
}

void updatePWMFreq(struct uMPPT *target_uMPPT, struct board_param* board, float new_freq){ //Frequency in kHz
	//Wrapper function to update the PWM frequency of the given uMPPT
	uint8_t i = target_uMPPT->pwm_num;

	if (i == 2){ //PWM3 is the only one on clock APB1
		board->uMPPTs[i]->pwm_frequency = new_freq;
		board->PWM_timers[i]->Instance->ARR = (uint32_t) (APB1_Initial_clk_freq/new_freq);
		updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle);
	} else if (i == 0 || i == 3){ //PWM1 and PWM4 share the same timer, so they both have to have the same frequency
		board->uMPPTs[0]->pwm_frequency = new_freq;
		board->uMPPTs[3]->pwm_frequency = new_freq;
		board->PWM_timers[i]->Instance->ARR = (uint32_t) (APB2_Initial_clk_freq/new_freq);
		updatePWMDutyCycle(board->uMPPTs[0], board, board->uMPPTs[0]->pwm_duty_cycle);
		updatePWMDutyCycle(board->uMPPTs[3], board, board->uMPPTs[3]->pwm_duty_cycle);
	} else {
		board->uMPPTs[i]->pwm_frequency = new_freq;
		board->PWM_timers[i]->Instance->ARR = (uint32_t) (APB2_Initial_clk_freq/new_freq);
		updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle);
	}
}


