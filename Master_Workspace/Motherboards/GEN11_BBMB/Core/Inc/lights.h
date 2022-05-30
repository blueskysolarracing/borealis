/*
 * TMC5160_driver.h
 *
 *  Created on: May 24, 2022
 *      Author: nkusanda
 */

#include "main.h"
#include "TMC5160_driver.h"

struct lights_stepper_ctrl{
	//SPI CS pins for TMC5160
	GPIO_TypeDef* CSPort0;
	uint16_t CSPin0;
	GPIO_TypeDef* CSPort1;
	uint16_t CSPin1;

	//Power EN for retractable light motor controller
	GPIO_TypeDef* PWR_EN_Port;
	uint16_t PWR_EN_Pin;

	//TMC5160 SPI interface
	SPI_HandleTypeDef* TMC5160_SPI;

	//LED TIM CH
	uint32_t master_CH;
	uint32_t left_ind_CH;
	uint32_t right_ind_CH;
	uint32_t DRL_CH;
	uint32_t BRK_CH;
	uint32_t FLT_CH;

	//LED TIM
	TIM_HandleTypeDef* master_TIM;
	TIM_HandleTypeDef* left_ind_TIM;
	TIM_HandleTypeDef* right_ind_TIM;
	TIM_HandleTypeDef* DRL_TIM;
	TIM_HandleTypeDef* BRK_TIM;
	TIM_HandleTypeDef* FLT_TIM;
};

void turn_on_indicators(struct lights_stepper_ctrl* lights, int left_or_right, double pwm_duty_cycle, double blink_rate, double on_period);
void turn_off_indicators(struct lights_stepper_ctrl* lights, int left_or_right);
void turn_on_DRL(struct lights_stepper_ctrl* lights, double pwm_duty_cycle);
void turn_off_DRL(struct lights_stepper_ctrl* lights);
void turn_on_brake_lights(struct lights_stepper_ctrl* lights, double pwm_duty_cycle);
void turn_off_brake_lights(struct lights_stepper_ctrl* lights);
void turn_on_hazard_lights(struct lights_stepper_ctrl* lights, double pwm_duty_cycle, double blink_rate);
void turn_off_hazard_lights(struct lights_stepper_ctrl* lights);
void turn_on_fault_indicator(struct lights_stepper_ctrl* lights);
void turn_off_fault_indicator(struct lights_stepper_ctrl* lights);


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
