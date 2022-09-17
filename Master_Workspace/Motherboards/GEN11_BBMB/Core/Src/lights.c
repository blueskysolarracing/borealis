/*
 * lights.c
 *
 *  Created on: May 24, 2022
 *      Author: nkusanda
 *
 *  All functions need to control the lighting system (GEN11) from BBMB
 */

#include "lights.h"

void turn_on_indicators(struct lights_stepper_ctrl* lights, int left_or_right, float pwm_duty_cycle){
	if (lights->hazard_state == LIGHTS_ON){ return; } //If hazards are on, don't do anything

	if (USE_RETRACTABLE_LIGHTS){
		HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board
		setup_motor(lights->TMC5160_SPI);
	}

	lights->ind_master_TIM->Instance->ARR = 63999; //Period of 1s (60 blinks per second) assuming 64MHz clock and 999 prescaler.
	lights->ind_master_TIM->Instance->CCR1 = 32000; //On-time of 50%
	lights->left_ind_TIM->Instance->ARR = 63; //Period of 1ms assuming 64MHz clock and 999 prescaler
	//(Slave ARR + 1) has to be a multiple of (Master ARR + 1) for timing to match

	//turn on master timer
	HAL_TIM_PWM_Start(lights->ind_master_TIM, lights->ind_master_CH);

	if (left_or_right == LEFT){
		if (USE_RETRACTABLE_LIGHTS){	rotate(0, LEFT, lights->TMC5160_SPI); 	}// anti-clockwise: out for left
		lights->left_ind_TIM->Instance->CCR1 = pwm_duty_cycle * lights->left_ind_TIM->Instance->ARR;
		HAL_TIM_PWM_Start(lights->left_ind_TIM, lights->left_ind_CH);

	} else if (left_or_right == RIGHT){
		if (USE_RETRACTABLE_LIGHTS){	rotate(1, RIGHT, lights->TMC5160_SPI); 	}// clockwise: out for right
		lights->right_ind_TIM->Instance->CCR2 = pwm_duty_cycle * lights->left_ind_TIM->Instance->ARR;
		HAL_TIM_PWM_Start(lights->right_ind_TIM, lights->right_ind_CH);
	}
}

void turn_off_indicators(struct lights_stepper_ctrl* lights, int left_or_right){
	if (lights->hazard_state == LIGHTS_ON){ return; } //If hazards are on, don't do anything

	if (USE_RETRACTABLE_LIGHTS){
		HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board
		setup_motor(lights->TMC5160_SPI);
	}

	if (left_or_right == LEFT){ //left
		HAL_TIM_PWM_Stop(lights->left_ind_TIM, lights->left_ind_CH);
		if (USE_RETRACTABLE_LIGHTS){	rotate(1, 0, lights->TMC5160_SPI);	} // clockwise: in for left
	}

	else { //right
		HAL_TIM_PWM_Stop(lights->right_ind_TIM, lights->right_ind_CH);
		if (USE_RETRACTABLE_LIGHTS){	rotate(0, 1, lights->TMC5160_SPI);	} // anti-clockwise: in for right
	}
}

void turn_on_DRL(struct lights_stepper_ctrl* lights, float pwm_duty_cycle){
	lights->DRL_TIM->Instance->ARR = 63; //1ms period
	lights->DRL_TIM->Instance->CCR1 = pwm_duty_cycle * lights->DRL_TIM->Instance->ARR;

	if (USE_RETRACTABLE_LIGHTS){
		HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board
		setup_motor(lights->TMC5160_SPI);

		rotate(0, 0, lights->TMC5160_SPI); // anti-clockwise: out for left
		rotate(1, 1, lights->TMC5160_SPI); // clockwise: out for right
	}

	HAL_TIM_PWM_Start(lights->DRL_TIM, lights->DRL_CH); // Channel 1 is DRL
}

void turn_off_DRL(struct lights_stepper_ctrl* lights){
	if (USE_RETRACTABLE_LIGHTS){
		HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board
		rotate(0, 1, lights->TMC5160_SPI); // clockwise: in for right
		rotate(1, 0, lights->TMC5160_SPI); // anti-clockwise: in for left
	}

	HAL_TIM_PWM_Stop(lights->DRL_TIM, lights->DRL_CH); // Channel 1 is DRL
}

void turn_on_brake_lights(struct lights_stepper_ctrl* lights, float pwm_duty_cycle){
	lights->BRK_TIM->Instance->ARR = 63; //Period of 1ms
	lights->BRK_TIM->Instance->CCR2 = pwm_duty_cycle * lights->BRK_TIM->Instance->ARR;
	HAL_TIM_PWM_Start(lights->BRK_TIM, lights->BRK_CH); // Channel 2 is brake lights
}

void turn_off_brake_lights(struct lights_stepper_ctrl* lights){
	HAL_TIM_PWM_Stop(lights->BRK_TIM, lights->BRK_CH); // Channel 2 is brake lights
}

void turn_on_hazard_lights(struct lights_stepper_ctrl* lights, float pwm_duty_cycle){

	turn_on_indicators(lights, LEFT, pwm_duty_cycle);
	turn_on_indicators(lights, RIGHT, pwm_duty_cycle);

	lights->hazard_state = LIGHTS_ON;
}

void turn_off_hazard_lights(struct lights_stepper_ctrl* lights){
	lights->hazard_state = LIGHTS_OFF;
	turn_off_indicators(lights, LEFT);
	turn_off_indicators(lights, RIGHT);
}

void turn_on_fault_indicator(struct lights_stepper_ctrl* lights, float pwm_duty_cycle){
	lights->FLT_master_TIM->Instance->ARR = 31999; //Period of 0.5s (120 blinks per second) assuming 64MHz clock and 999 prescaler.
	lights->FLT_master_TIM->Instance->CCR1 = 16000; //On-time of 50%
	//(Slave ARR + 1) has to be a multiple of (Master ARR + 1) for timing to match
	HAL_TIM_PWM_Start(lights->FLT_master_TIM, lights->FLT_master_CH);

	lights->FLT_TIM->Instance->ARR = 31; //Period of 0.5ms assuming 64MHz clock and 999 prescaler
	lights->FLT_TIM->Instance->CCR1 = pwm_duty_cycle * lights->FLT_TIM->Instance->ARR;

	HAL_TIM_PWM_Start(lights->FLT_TIM, lights->FLT_CH);
}

void turn_off_fault_indicator(struct lights_stepper_ctrl* lights){
//	// default initialization is okay
//	lights->FLT_TIM->Instance->CCR1 = 0;
	HAL_TIM_PWM_Stop(lights->FLT_TIM, lights->FLT_CH);
}

