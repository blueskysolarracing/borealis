/*
 * lights.c
 *
 *  Created on: May 24, 2022
 *      Author: nkusanda
 *
 *  All functions need to control the lighting system (GEN11) from BBMB
 */

#include "lights.h"

void turn_on_indicators(struct lights_stepper_ctrl* lights, int left_or_right, double pwm_duty_cycle, double blink_rate, double on_period)
{
	double frequency = blink_rate / 60; // blink_rate given in bpm
	double period_master = frequency * 75000; // based on internal clock, pre-scaler (assumes 75MHz TIM2 clock frequency)

	double pulse_slave = on_period; // 160 ticks
	double period_slave = on_period / pwm_duty_cycle; // 160 / 0.10 = 1600
	HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board

	// setup_motor(&hspi5, left_or_right);
	setup_motor(lights->TMC5160_SPI);

	lights->master_TIM->Init.Period = period_master;
	lights->left_ind_TIM->Init.Period = period_slave;
	lights->left_ind_TIM->Instance->CCR1 = on_period;
//	my_MX_TIM1_Init(period_master);
//	my_MX_TIM2_Init(pulse_slave, period_slave);

	//turn on master timer
	HAL_TIM_PWM_Start(lights->master_TIM, lights->master_CH);

	if (left_or_right == 0){
		rotate(0, 0, lights->TMC5160_SPI); // anti-clockwise: out for left
		HAL_TIM_PWM_Start(lights->left_ind_TIM, lights->left_ind_CH);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Channel 1 is left light
	}

	else{
		rotate(1, 1, lights->TMC5160_SPI); // clockwise: out for right
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Channel 2 is right light
		HAL_TIM_PWM_Start(lights->right_ind_TIM, lights->right_ind_CH);
	}
	//HAL_Delay(5000);
	//HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5, 0); // write to depower board
}

void turn_off_indicators(struct lights_stepper_ctrl* lights, int left_or_right){
	HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board
	//setup_motor(&hspi5);

	if (left_or_right == 0){ //left
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(lights->left_ind_TIM, lights->left_ind_CH);
		rotate(1, 0, lights->TMC5160_SPI); // clockwise: in for left
	}

	else{ //right
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(lights->right_ind_TIM, lights->right_ind_CH);
		rotate(0, 1, lights->TMC5160_SPI); // anti-clockwise: in for right
	}

	HAL_TIM_PWM_Stop(lights->master_TIM, lights->master_CH); //Master timer
//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);// master timer - need to turn off?
}


void turn_on_DRL(struct lights_stepper_ctrl* lights, double pwm_duty_cycle)
{
	double pulse = pwm_duty_cycle * 1600;

	HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board
	setup_motor(lights->TMC5160_SPI);

	rotate(0, 0, lights->TMC5160_SPI); // anti-clockwise: out for left
	rotate(1, 1, lights->TMC5160_SPI); // clockwise: out for right

	my_MX_TIM5_Init(pulse);
	HAL_TIM_PWM_Start(lights->DRL_TIM, lights->DRL_CH); // Channel 1 is DRL
}

void turn_off_DRL(struct lights_stepper_ctrl* lights){
	HAL_TIM_PWM_Stop(lights->DRL_TIM, lights->DRL_CH);

	HAL_GPIO_WritePin(lights->PWR_EN_Port, lights->PWR_EN_Pin, 1); // write to power board
	rotate(0, 1, lights->TMC5160_SPI); // clockwise: in for right
	rotate(1, 0, lights->TMC5160_SPI); // anti-clockwise: in for left
}

void turn_on_brake_lights(struct lights_stepper_ctrl* lights, double pwm_duty_cycle)
{
	double pulse = pwm_duty_cycle * 1600;

	my_MX_TIM5_Init(pulse);
	HAL_TIM_PWM_Start(lights->BRK_TIM, lights->BRK_CH); // Channel 2 is brake lights
}

void turn_off_brake_lights(struct lights_stepper_ctrl* lights){
	HAL_TIM_PWM_Stop(lights->BRK_TIM, lights->BRK_CH);
}

void turn_on_hazard_lights(struct lights_stepper_ctrl* lights, double pwm_duty_cycle, double blink_rate)
{
	double frequency = blink_rate / 60; // blink_rate given in bpm
	double period_master = frequency * 75000; // based on internal clock, pre-scaler (assumes 75MHz TIM2 clock frequency)

	double on_period = 160; // Constant for hazard lights
	double pulse_slave = on_period; // 160
	double period_slave = on_period / pwm_duty_cycle; // 160 * 10 = 1600

	my_MX_TIM1_Init(period_master);
	my_MX_TIM2_Init(pulse_slave, period_slave);

	HAL_TIM_PWM_Start(lights->master_TIM, lights->master_CH); //master
	HAL_TIM_PWM_Start(lights->left_ind_TIM, lights->left_ind_CH); // Both left and right lights
	HAL_TIM_PWM_Start(lights->right_ind_TIM, lights->right_ind_CH);
}

void turn_off_hazard_lights(struct lights_stepper_ctrl* lights)
{

	HAL_TIM_PWM_Start(lights->master_TIM, lights->master_CH); //master
	HAL_TIM_PWM_Stop(lights->left_ind_TIM, lights->left_ind_CH); // Both left and right lights
	HAL_TIM_PWM_Stop(lights->right_ind_TIM, lights->right_ind_CH);
}

void turn_on_fault_indicator(struct lights_stepper_ctrl* lights)
{
	// default initialization is okay
	HAL_TIM_PWM_Start(lights->FLT_TIM, lights->FLT_CH);
}

void turn_off_fault_indicator(struct lights_stepper_ctrl* lights)
{
	// default initialization is okay
	HAL_TIM_PWM_Stop(lights->FLT_TIM, lights->FLT_CH);
}
