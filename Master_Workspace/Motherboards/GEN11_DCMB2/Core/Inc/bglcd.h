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
	uint8_t battery_relay_state = 255;
	uint8_t array_relay_state = 255;

	short 	solar_power;
	short 	motor_power;
	short	battery_power;
	short	LV_power;

	short	LV_voltage;

	uint8_t	battery_soc;
};

struct disp_default_frame{
	uint8_t P1_speed_kph;
	uint8_t	P1_left_indicator_status;
	// 0 rev, 1 fwd
	uint8_t direction;
	uint8_t eco;
	uint8_t light;
	uint8_t batt_warning;
	uint8_t hazard;

	uint8_t	P2_DRL_state;
	uint8_t	P2_motor_state;
	uint8_t	P2_VFM;
	uint8_t	P2_right_indicator_status;
	uint8_t P2_low_supp_volt;
};

struct disp_detailed_frame{
	short 	P1_solar_voltage;
	float 	P1_solar_current;
	short 	P1_motor_voltage;
	float 	P1_motor_current;
	short 	P1_battery_voltage;
	float 	P1_battery_current;

	short	P2_HV_voltage;
	short	P2_LV_current;
	short	P2_max_batt_temp;

	uint8_t	P2_BB;
	uint8_t	P2_MC;
	uint8_t	P2_BMS;
	uint8_t	P2_PPT;
	uint8_t	P2_RAD;

	uint8_t faultType; // see enum BATTERY_FAULT_TYPE
	uint8_t faultCell;
	uint8_t faultTherm;

	uint32_t overvoltage_status;
	uint32_t undervoltage_status;
	uint32_t overtemperature_status;
	uint32_t undertemperature_status;
	uint8_t overcurrent_status;

	int32_t min_temperature;
	int32_t min_temperature_cell;
	int32_t max_temperature;
	int32_t max_temperature_cell;
	int32_t min_voltage;
	int32_t min_voltage_cell;
	int32_t max_voltage;
	int32_t max_voltage_cell;
	int32_t min_soc;
	int32_t min_soc_cell;
	int32_t max_soc;
	int32_t max_soc_cell;
};

extern struct disp_common common_data;
extern struct disp_default_frame default_data;
extern struct disp_detailed_frame detailed_data;

enum DRIVING_DIRECTION {
	REVERSE,
	FORWARD
};

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
void drawLogo();
#endif /* BGLCD_H_ */
