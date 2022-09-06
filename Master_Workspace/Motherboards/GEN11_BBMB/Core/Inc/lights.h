/*
 * TMC5160_driver.h
 *
 *  Created on: May 24, 2022
 *      Author: nkusanda
 */

#include "main.h"
#include "TMC5160_driver.h"

enum LIGHTS_STATE {
	LIGHTS_OFF,
	LIGHTS_ON,
};

enum LIGHTS_INDICATOR {
	LEFT,
	RIGHT,
};

enum LIGHTS {
	LIGHTS_UNUSED,
	INDICATOR_LIGHTS = 0x02,
	DRL_LIGHTS = 0x04,
	BRAKES_LIGHTS = 0x8,
	HAZARD_LIGHTS = 0x10,
	FAULT_LIGHTS = 0x20
};
#define USE_RETRACTABLE_LIGHTS 0 //Set to 1 to enable stepper motors for retractable front lights

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
	uint32_t ind_master_CH;
	uint32_t left_ind_CH;
	uint32_t right_ind_CH;
	uint32_t DRL_CH;
	uint32_t BRK_CH;
	uint32_t FLT_CH;
	uint32_t FLT_master_CH;

	//LED TIM
	TIM_HandleTypeDef* ind_master_TIM;
	TIM_HandleTypeDef* left_ind_TIM;
	TIM_HandleTypeDef* right_ind_TIM;
	TIM_HandleTypeDef* DRL_TIM;
	TIM_HandleTypeDef* BRK_TIM;
	TIM_HandleTypeDef* FLT_TIM;
	TIM_HandleTypeDef* FLT_master_TIM;

	//States
	uint8_t hazard_state;
};

void turn_on_indicators(struct lights_stepper_ctrl* lights, int left_or_right, float pwm_duty_cycle);
void turn_off_indicators(struct lights_stepper_ctrl* lights, int left_or_right);
void turn_on_DRL(struct lights_stepper_ctrl* lights, float pwm_duty_cycle);
void turn_off_DRL(struct lights_stepper_ctrl* lights);
void turn_on_brake_lights(struct lights_stepper_ctrl* lights, float pwm_duty_cycle);
void turn_off_brake_lights(struct lights_stepper_ctrl* lights);
void turn_on_hazard_lights(struct lights_stepper_ctrl* lights, float pwm_duty_cycle);
void turn_off_hazard_lights(struct lights_stepper_ctrl* lights);
void turn_on_fault_indicator(struct lights_stepper_ctrl* lights, float pwm_duty_cycle);
void turn_off_fault_indicator(struct lights_stepper_ctrl* lights);
