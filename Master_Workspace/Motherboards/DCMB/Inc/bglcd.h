/*
 * bglcd.h
 *
 *  Created on: May 28, 2022
 *      Author: tristan
 */

#ifndef BGLCD_H_
#define BGLCD_H_

//Structs containing data to display on driver displays
struct disp_common{
	uint16_t 	solar_power;
	uint16_t 	motor_power;
	uint16_t	battery_power;
	uint16_t	LV_power;

	uint16_t 	solar_voltage;
	uint16_t	LV_voltage;

	uint8_t		P1_battery_soc;
};

struct disp_default_frame{
	uint8_t 	P1_speed_kph;
	uint8_t		P1_left_indicator_status;

	uint8_t		P2_cruise_state;
	uint8_t		P2_DRL_state;
	uint8_t		P2_regen_state;
	uint8_t		P2_left_indicator_status;
};

struct disp_detailed_frame{
	uint16_t 	P1_solar_voltage;
	uint16_t 	P1_solar_current;
	uint16_t 	P1_motor_voltage;
	uint16_t 	P1_motor_current;

	uint16_t	P2_LV_voltage;
	uint16_t	P2_LV_current;
	uint16_t	P2_max_batt_temp;
	uint16_t	P2_VFM_state;
};

void drawP1Default(int value[4]);
void drawP1Detailed(int value[9]);
void drawP1Activate();
void drawP1Deactivate();
void drawP1IgnitionOff();
void drawP1BMSFault();
void drawP1(uint8_t sel);
void drawP2Default(int value[4]);
void drawP2Detailed(int value[5]);
void drawP2Activate();
void drawP2Deactivate();
void drawP2IgnitionOff(int value[4]);
void drawP2BMSFault(int value[4]);
void drawP2(uint8_t sel);
#endif /* BGLCD_H_ */
