#include "uMPPT_STM32L071RBTx.h"


void config_uMPPT(struct board_param* board){
	/*! \brief Set initial signals and configure PWM, ADC
	 */

	//Initialize structs
	for (int i = 0; i < NUM_UMPPT; i++){
		board->uMPPT_list[i]->pwm_num = i;
//		board->uMPPT_list[i]->pwm_duty_cycle = DEFAULT_PWM_DUTY_CYCLE;
//		board->uMPPT_list[i]->pwm_frequency = DEFAULT_PWM_FREQ;
		updatePWMDutyCycle(board->uMPPT_list[i], board, DEFAULT_PWM_DUTY_CYCLE);
		updatePWMFreq(board->uMPPT_list[i], board, DEFAULT_PWM_FREQ);
	}

	//Enable MCU_OK_LED interrupt
	NVIC_EnableIRQ(TIM6_IRQn);
	HAL_TIM_Base_Start_IT((TIM_HandleTypeDef*) board->MCU_OK_LED_TIM);

	//Start ADC
	HAL_ADCEx_Calibration_Start(board->hadc_handle, ADC_SINGLE_ENDED);

	//Remove from sequencer
	for (int i = 0; i < 7; i++){
		board->ADC_config->Channel = board->ADC_CH[i];
		board->ADC_config->Rank = ADC_RANK_NONE;
		HAL_ADC_ConfigChannel(board->hadc_handle, board->ADC_config);
	}

	//Start PWM
	for (int i = 0; i < NUM_UMPPT; i++){
		HAL_TIM_PWM_Start(board->PWM_TIM[i], board->PWM_CHANNEL[i]);
	}
}

float uMPPT_read_ADC(uint32_t CH, struct board_param* board){
	/*! \brief Returns raw reading from specified ADC channel and, optionally, print it to UART port.
	 * 	CH is ADC_CHANNEL_x; will print to serial port if print_EN == 1
	 */

	float ADC_Value = 0;

	//Configure current channel on sequencer
	board->ADC_config->Channel = CH;
	board->ADC_config->Rank = ADC_RANK_CHANNEL_NUMBER;
	HAL_ADC_ConfigChannel(board->hadc_handle, board->ADC_config);

	//Start ADC
	HAL_ADC_Start(board->hadc_handle);
	HAL_ADC_PollForConversion(board->hadc_handle, 100);

	//Get ADC value
	ADC_Value = HAL_ADC_GetValue(board->hadc_handle);

	//Remove current channel from sequencer
	board->ADC_config->Rank = ADC_RANK_NONE;
	HAL_ADC_ConfigChannel(board->hadc_handle, board->ADC_config);

	//Print to serial port if enabled
	if (ADC_DEBUG_FLAG){
		char string[100] = {0};
		sprintf(string, "ADC reading for CH %ld: %f\n", CH, ADC_Value);
		HAL_UART_Transmit(board->huart_handle, (uint8_t *) string, strlen(string), 10);
	}

	return ADC_Value;
}

void update_MPP_IncCond(struct board_param *board, struct uMPPT *target_uMPPT){
	uint8_t found_MPP = 0;
	char print_str[100] = {0};

	//-----
	//NOT TESTED
	//-----

	target_uMPPT->MPPT_in_progress = 1; //Update flag to tell MPPT is in progress

	//This is a single loop in the MPPT algorithm. Repeats until MPP is found (dI/dV = -I/V)
	do {
		float raw_ADC = 0.0;

		//Update uMPPT current
		for (int i = 0; i < NUM_AVG_CURRENT; i++){
			raw_ADC += uMPPT_read_ADC(board->ADC_CH[6], board); //Measure output current
		}

		raw_ADC /= NUM_AVG_CURRENT;

		board->I_Out = (((raw_ADC / 4095.0) * VDDA) - I_MEAS_OFFSET) / I_SENSE_AMP_RATIO / I_SHUNT_VALUE;
		target_uMPPT->calc_input_current = board->I_Out * target_uMPPT->pwm_duty_cycle;

		//Update uMPPT voltage
		raw_ADC = uMPPT_read_ADC(board->ADC_CH[target_uMPPT->pwm_num], board);
		target_uMPPT->prev_input_voltage = target_uMPPT->input_voltage;
		target_uMPPT->input_voltage = (raw_ADC / 4095.0) * 2.0 * VDDA;

		if (DEBUG_FLAG){ //Print measurements
			sprintf(print_str, "Output current: %fA\n", board->I_Out);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
			sprintf(print_str, "uMPPT #%d input voltage: %fV\n", target_uMPPT->pwm_num + 1, target_uMPPT->input_voltage);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
			sprintf(print_str, "uMPPT #%d input current: %fA\n", target_uMPPT->pwm_num + 1, target_uMPPT->calc_input_current);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
			sprintf(print_str, "uMPPT #%d input power: %fW\n\n", target_uMPPT->pwm_num + 1, target_uMPPT->input_voltage * target_uMPPT->calc_input_current);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
		}

		//Incremental conductance decision (based on: https://ieeexplore.ieee.org/document/5456214)
		float inc_voltage = target_uMPPT->input_voltage - target_uMPPT->prev_input_voltage;
		float inc_current = target_uMPPT->calc_input_current - target_uMPPT->prev_calc_input_current;

		if ((inc_voltage < board->dV_error) && (inc_voltage > (-ALLOWED_IncCond_dIdV_ERROR))){ //Equivalent to dV == 0, considering errors
			//Yes; dV == 0
			if ((inc_current < board->dI_error) && (inc_current > (-ALLOWED_IncCond_dIdV_ERROR))){
				//Yes; dI == 0
				//No change needed and we are at MPP
				//Update flag and MPP voltage/current
				found_MPP = 1;
				if (DEBUG_FLAG){	HAL_UART_Transmit(board->huart_handle, (uint8_t *) "Found MPP!\n", 12, 10);	}
				target_uMPPT->MPP_voltage = target_uMPPT->input_voltage;
				target_uMPPT->MPP_current = target_uMPPT->calc_input_current;
			} else {
				//No; dI != 0
				if (inc_current > ALLOWED_IncCond_dIdV_ERROR){
					//Yes; dI > 0 --> Increase duty cycle
					updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle + PWM_DUTY_CYCLE_STEP_CHANGE);
				} else {
					//No; dI < 0 --> Decrease duty cycle
					updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle - PWM_DUTY_CYCLE_STEP_CHANGE);
				}
			}
		} else {
			//No; dV != 0
			float inc_cond = inc_current / inc_voltage; //Calculate incremental conductance
			float current_cond = target_uMPPT->calc_input_current / target_uMPPT->input_voltage;

			if ((inc_cond < (-current_cond + ALLOWED_IncCond_dIdV_ERROR)) && (inc_cond > (-current_cond - ALLOWED_IncCond_dIdV_ERROR))){
				//Yes; dI/dV = -I/V
				//No change needed and we are at MPP
				//Update flag and MPP voltage/current
				found_MPP = 1;
				if (DEBUG_FLAG){	HAL_UART_Transmit(board->huart_handle, (uint8_t *) "Found MPP!\n", 12, 10);	}
				target_uMPPT->MPP_voltage = target_uMPPT->input_voltage;
				target_uMPPT->MPP_current = target_uMPPT->calc_input_current;
			} else {
				//No; dI/dV != -I/V
				if (inc_cond > -current_cond + ALLOWED_IncCond_dIdV_ERROR){
					//Yes; dI/dV > -I/V --> Increase duty cycle
					updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle + PWM_DUTY_CYCLE_STEP_CHANGE);
				} else {
					//No; dI/dV < -I/V --> Decrease duty cycle
					updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle - PWM_DUTY_CYCLE_STEP_CHANGE);
				}
			}
		}

		if (DEBUG_FLAG){
			sprintf(print_str, "PWM duty cycle: %f\n\n", target_uMPPT->pwm_duty_cycle);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
		}

		HAL_Delay(MPPT_ITERATION_DELAY);
	} while (found_MPP != 1);

	target_uMPPT->MPPT_in_progress = 0; //Update flag to tell MPPT is not in progress

	if (DEBUG_FLAG){ HAL_UART_Transmit(board->huart_handle, (uint8_t *) "\n\n\n\n\n", 6, 10); }
}

void update_MPP_HillClimb(struct board_param *board, struct uMPPT *target_uMPPT){
	uint8_t found_MPP = 0;
	char print_str[100] = {0};

	target_uMPPT->MPPT_in_progress = 1; //Update flag to tell MPPT is in progress

	//This is a single loop in the MPPT algorithm. Repeats until MPP is found
	do {
		float raw_ADC = 0.0;
		float prev_power = 0;
		float current_power = 0;

		//Update uMPPT current
		for (int i = 0; i < NUM_AVG_CURRENT; i++){
			raw_ADC += uMPPT_read_ADC(board->ADC_CH[6], board); //Measure output current
		}

		raw_ADC /= NUM_AVG_CURRENT;

		board->I_Out = (((raw_ADC / 4095.0) * VDDA) - I_MEAS_OFFSET) / I_SENSE_AMP_RATIO / I_SHUNT_VALUE;
		target_uMPPT->calc_input_current = board->I_Out * target_uMPPT->pwm_duty_cycle;

		//Update uMPPT voltage
		raw_ADC = uMPPT_read_ADC(board->ADC_CH[target_uMPPT->pwm_num], board);
		target_uMPPT->prev_input_voltage = target_uMPPT->input_voltage;
		target_uMPPT->input_voltage = (raw_ADC / 4095.0) * 2.0 * VDDA;

		if (DEBUG_FLAG){ //Print measurements
			sprintf(print_str, "Output current: %fA\n", board->I_Out);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
			sprintf(print_str, "uMPPT #%d input voltage: %fV\n", target_uMPPT->pwm_num + 1, target_uMPPT->input_voltage);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
			sprintf(print_str, "uMPPT #%d input current: %fA\n", target_uMPPT->pwm_num + 1, target_uMPPT->calc_input_current);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
			sprintf(print_str, "uMPPT #%d input power: %fW\n\n", target_uMPPT->pwm_num + 1, target_uMPPT->input_voltage * target_uMPPT->calc_input_current);
			HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
		}

		//Hill climbing algorithm
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

void calib_current_shunt_ADC(){
	//Needed to determine the voltage offset at current shunt ADC pin in zero-current condition
}

void updatePWMDutyCycle(struct uMPPT *target_uMPPT, struct board_param* board, float new_duty_cycle){
	if (DEBUG_FLAG){
		char print_str[50] = {0};
		sprintf(print_str, "New duty cycle requested: %f\n", new_duty_cycle);
		HAL_UART_Transmit(board->huart_handle, (uint8_t *) print_str, strlen(print_str), 10);
	}

	//Bound PWM duty cycle
	if (new_duty_cycle < MIN_PWM_DUTY_CYCLE){ new_duty_cycle = MIN_PWM_DUTY_CYCLE; }
	else if (new_duty_cycle > MAX_PWM_DUTY_CYCLE){ new_duty_cycle = MAX_PWM_DUTY_CYCLE; }

	//Update appropriate timer register
	if (target_uMPPT->pwm_num <= 1){ //If uMPPT 1 or 2
		board->PWM_TIM[target_uMPPT->pwm_num]->Instance->CCR1 = round(new_duty_cycle * board->PWM_TIM[target_uMPPT->pwm_num]->Instance->ARR);
	} else if (target_uMPPT->pwm_num >= 3){ //If uMPPT 4 or 5
		board->PWM_TIM[target_uMPPT->pwm_num]->Instance->CCR3 = round(new_duty_cycle * board->PWM_TIM[target_uMPPT->pwm_num]->Instance->ARR);
	} else { //If uMPPT #2
		board->PWM_TIM[target_uMPPT->pwm_num]->Instance->CCR4 = round(new_duty_cycle * board->PWM_TIM[target_uMPPT->pwm_num]->Instance->ARR);
	}
	target_uMPPT->pwm_duty_cycle = new_duty_cycle;
}

void updatePWMPhaseOffset(struct uMPPT *target_uMPPT, struct board_param* board, float new_phase_offset){
	//Empty for now
}

void updatePWMFreq(struct uMPPT *target_uMPPT, struct board_param* board, float new_freq){ //Frequency in kHz
	//Wrapper function to update the PWM frequency of the given uMPPT
	uint8_t i = target_uMPPT->pwm_num;

	if (i == 2){ //PWM3 is the only one on clock APB1
		board->uMPPT_list[i]->pwm_frequency = new_freq;
		board->PWM_TIM[i]->Instance->ARR = (uint32_t) (APB1_Initial_clk_freq/new_freq);
		updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle);
	} else if (i == 0 || i == 3){ //PWM1 and PWM4 share the same timer, so they both have to have the same frequency
		board->uMPPT_list[0]->pwm_frequency = new_freq;
		board->uMPPT_list[3]->pwm_frequency = new_freq;
		board->PWM_TIM[i]->Instance->ARR = (uint32_t) (APB2_Initial_clk_freq/new_freq);
		updatePWMDutyCycle(board->uMPPT_list[0], board, board->uMPPT_list[0]->pwm_duty_cycle);
		updatePWMDutyCycle(board->uMPPT_list[3], board, board->uMPPT_list[3]->pwm_duty_cycle);
	} else {
		board->uMPPT_list[i]->pwm_frequency = new_freq;
		board->PWM_TIM[i]->Instance->ARR = (uint32_t) (APB2_Initial_clk_freq/new_freq);
		updatePWMDutyCycle(target_uMPPT, board, target_uMPPT->pwm_duty_cycle);
	}
}
