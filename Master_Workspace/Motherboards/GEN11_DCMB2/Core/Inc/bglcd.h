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
	short 	solar_power;
	short 	motor_power;
	short	battery_power;
	short	LV_power;

	short 	solar_voltage;
	short	LV_voltage;

	uint8_t	battery_soc;
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
	short 	P1_solar_voltage;
	short 	P1_solar_current;
	short 	P1_motor_voltage;
	short 	P1_motor_current;
	short 	P1_battery_voltage;
	short 	P1_battery_current;

	short	P2_HV_voltage;
	short	P2_LV_voltage;
	short	P2_LV_current;
	short	P2_max_batt_temp;
	short	P2_VFM_state;
};

extern disp_common common_data;
extern disp_default_frame default_data;
extern disp_detailed_frame detailed_data;

void drawP1Default(/*int value[4]*/);
void drawP1Detailed(/*int value[9]*/);
void drawP1Activate();
void drawP1Deactivate();
void drawP1IgnitionOff();
void drawP1BMSFault();
void drawP1(uint8_t sel);
void drawP2Default(/*int value[4]*/);
void drawP2Detailed(/*int value[5]*/);
void drawP2Activate();
void drawP2Deactivate();
void drawP2IgnitionOff(/*int value[4]*/);
void drawP2BMSFault(/*int value[4]*/);
void drawP2(uint8_t sel);
#endif /* BGLCD_H_ */
