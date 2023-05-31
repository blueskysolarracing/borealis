/*
 * bglcd.c
 *
 *	//Helpers to draw frames for GEN11 BSSR car
 *
 *  Created on: May 28, 2022
 *      Author: tristan
 */

#include <stdio.h>
#include <stdlib.h>
#include "bglcd.h"
#include "glcd.h"
#include "main.h"
#include "display_BSSR.h"
#include "driver_disp_icon.h"
#include "battery_config.h"

/** Fonts */
#include "fonts/font5x7.h"
#include "fonts/Liberation_Sans11x14_Numbers.h"
#include "fonts/Liberation_Sans15x21_Numbers.h"
#include "fonts/Liberation_Sans17x17_Alpha.h"
#include "fonts/Liberation_Sans27x36_Numbers.h"
#include "fonts/Liberation_Sans20x28_Numbers.h"
#include "fonts/Bebas_Neue20x36_Bold_Numbers.h"
#include "fonts/JetBrains_Mono13x20.h"
#include "fonts/JetBrains_Mono13x21_Symbol.h"
#include "fonts/font5x7.h"

uint8_t correct_Y(uint8_t y){
	if(pToggle)
		return y;	// change this if P2 fuck up
//	return (y+16)%64;
	return y;	// change this if P1 fuck up
}

// draw logo on both screens
void drawLogo(){
	pToggle = 0;
	glcd_clear_buffer();
	for(int i = 0; i < 64; i++){
		for(int j = 0; j < 128; j++){
			if(bssr_logo[i][j]){
				glcd_set_pixel(j, i, 1);
			}
		}
	}
	glcd_write();
	pToggle = 1;
	glcd_clear_buffer();
	for(int i = 0; i < 64; i++){
		for(int j = 0; j < 128; j++){
			if(bssr_logo[i][j]){
				glcd_set_pixel(j, i, 1);
			}
		}
	}
	glcd_write();
}

// p1 stuff start
void drawP1Default_new(/*int value[4]*/){
	short value[4] = {	common_data.solar_power,
						common_data.motor_power,
						common_data.battery_power,
						(short)(default_data.P1_speed_kph)};

	char labelsP1[3][14] = {0};
	int labelsP1L = 3;
	char* labelspeed = "km/h";

	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
	glcd_clear_buffer();

	// populate label
	if(value[0] >= 0) sprintf(labelsP1[0], "Solar: +%4dW\0", value[0]);
	else sprintf(labelsP1[0], "Solar:-%4dW\0", abs(value[0]));

	if(value[1] >= 0) sprintf(labelsP1[1], "Motor: +%4dW\0", value[1]);
	else sprintf(labelsP1[1], "Motor:-%4dW\0", abs(value[1]));

	if(value[2] >= 0) sprintf(labelsP1[2], "Batt:  +%4dW\0", value[2]);
	else sprintf(labelsP1[2], "Batt: -%4dW\0", abs(value[2]));

	// start drawing at y = 5
	uint8_t y = 5;

	// draw the labels
	for(int i = 0; i < labelsP1L; i++){
		char* label = labelsP1[i];
		int j = 0;
		// char by char cuz draw xy only with char
		while(label[j] != '\0'){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
			j++;
		}
		// go next rows, these value are just what I think will look good
		y+=15;
	}

	// draw icons
	y = 52;
	uint8_t x = 10;
	if(default_data.eco == 0){
		for(int i = 0; i < 12; i++){
			for(int j = 0; j < 12; j++){
				if(leaf[i][j])
				glcd_set_pixel(x+j, y+i, 1);
			}
		}
	}
	x = 32;
	if(default_data.direction){
		glcd_fill_rect(x, correct_Y(y-1), 13, 11, 1);
		glcd_tiny_draw_char_xy_white(x+4, correct_Y(y+1), 'F');
	}
	else{
		glcd_fill_rect(x, correct_Y(y-1), 13, 11, 1);
		glcd_tiny_draw_char_xy_white(x+4, correct_Y(y+1), 'R');
	}
	x = 54;
	if(default_data.light){
		for(int i = 0; i < 12; i++){
			for(int j = 0; j < 17; j++){
				if(headlight[i][j])
				glcd_set_pixel(x+j, y+i, 1);
			}
		}
	}

	// draw icon for battery and hazard
	x = 84;
	y = 0;
	if(default_data.batt_warning){
		for(int i = 0; i < 13; i++){
			for(int j = 0; j < 13; j++){
				if(batt[i][j])
				glcd_set_pixel(x+j, y+i, 1);
			}
		}
	}
	x = 106;
	if(default_data.hazard){
		for(int i = 0; i < 13; i++){
			for(int j = 0; j < 13; j++){
				if(hazard[i][j])
				glcd_set_pixel(x+j, y+i, 1);
			}
		}
	}

	// dir light
	x = 80;
	y = 50;
	if(default_data.P1_left_indicator_status == 0){
		glcd_tiny_draw_char_xy(x, correct_Y(y), '<');
		glcd_tiny_draw_char_xy(x+5, correct_Y(y), '-');
	}
	x = 116;
	if(default_data.P2_right_indicator_status == 0){
		glcd_tiny_draw_char_xy(x, correct_Y(y), '-');
		glcd_tiny_draw_char_xy(x+5, correct_Y(y), '>');
	}


	// draw divider line
	glcd_draw_line(79, 0, 79, 63, BLACK);
	x = 0;
	// draw km/h
	while(labelspeed[x] != 0){
		glcd_tiny_draw_char_xy(91+(x*6), correct_Y(50), labelspeed[x]);
		x++;
	}

	// now write the big speed
	if(value[3] < 100){
		glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
		if(value[3] >= 10)glcd_draw_char_xy(85, correct_Y(16), (value[3]/10) + '0');
		glcd_draw_char_xy(105, correct_Y(16), (value[3]%10) + '0');
	}
	else{
		glcd_set_font(JetBrains_Mono13x21_Symbol, 13, 21, ' ', '9');
		char valueS[4] = {0};
		sprintf(valueS, "%3d", value[3]);
		for(int i = 0; i < 3; i++){
			glcd_draw_char_xy(85+(i*13), correct_Y(19), valueS[i]);
		}
		// speed >= 100, case for three digits
	}

	glcd_write();
}

void drawP1Default(/*int value[4]*/){
	short value[4] = {	common_data.solar_power,
						common_data.motor_power,
						common_data.battery_power,
						(short)(default_data.P1_speed_kph)};

	char labelsP1[3][14] = {0};
	int labelsP1L = 3;
	char* labelspeed = "km/h";

	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
	glcd_clear_buffer();

	// populate label
	if(value[0] >= 0) sprintf(labelsP1[0], "Solar: +%4dW\0", value[0]);
	else sprintf(labelsP1[0], "Solar: -%4dW\0", abs(value[0]));

	if(value[1] >= 0) sprintf(labelsP1[1], "Motor: +%4dW\0", value[1]);
	else sprintf(labelsP1[1], "Motor: -%4dW\0", abs(value[1]));

	if(value[2] >= 0) sprintf(labelsP1[2], "Batt:  +%4dW\0", value[2]);
	else sprintf(labelsP1[2], "Batt:  -%4dW\0", abs(value[2]));

	// start drawing at y = 5
	uint8_t y = 5;

	// draw the labels
	for(int i = 0; i < labelsP1L; i++){
		char* label = labelsP1[i];
		int j = 0;
		// char by char cuz draw xy only with char
		while(label[j] != '\0'){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
			j++;
		}
		// go next rows, these value are just what I think will look good
		y+=15;
	}

	// draw icons
	y = 52;
	uint8_t x = 10;
	if(default_data.eco == 0){
		for(int i = 0; i < 12; i++){
			for(int j = 0; j < 12; j++){
				if(leaf[i][j])
				glcd_set_pixel(x+j, y+i, 1);
			}
		}
	}
	x = 32;
	if(default_data.direction){
		glcd_fill_rect(x, correct_Y(y-1), 13, 11, 1);
		glcd_tiny_draw_char_xy_white(x+4, correct_Y(y+1), 'F');
	}
	else{
		glcd_fill_rect(x, correct_Y(y-1), 13, 11, 1);
		glcd_tiny_draw_char_xy_white(x+4, correct_Y(y+1), 'R');
	}
	x = 54;
	if(default_data.light){
		for(int i = 0; i < 12; i++){
			for(int j = 0; j < 17; j++){
				if(headlight[i][j])
				glcd_set_pixel(x+j, y+i, 1);
			}
		}
	}


	// draw divider line
	glcd_draw_line(81, 0,  81, 63, BLACK);
	x = 0;
	// draw km/h
	while(labelspeed[x] != 0){
		glcd_tiny_draw_char_xy(94+(x*6), correct_Y(52), labelspeed[x]);
		x++;
	}

	// now write the big speed
	if(value[3] < 100){
		glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
		if(value[3] >= 10)glcd_draw_char_xy(85, correct_Y(16), (value[3]/10) + '0');
		glcd_draw_char_xy(105, correct_Y(16), (value[3]%10) + '0');
	}
	else{
		glcd_set_font(JetBrains_Mono13x21_Symbol, 13, 21, ' ', '9');
		char valueS[4] = {0};
		sprintf(valueS, "%3d", value[3]);
		for(int i = 0; i < 3; i++){
			glcd_draw_char_xy(85+(i*13), correct_Y(19), valueS[i]);
		}
		// speed >= 100, case for three digits
	}

	glcd_write();
}

void drawP1Activate(){
	char* labels[] = {"CRUISE CONTROL", "ACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 17;
    uint8_t x = 22;

    char* ptr = labels[0];
    while(*ptr){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    y = 40;
    x = 37;
    // play with this
    glcd_fill_rect(30, correct_Y(y-1), 68, 9, 1);

    ptr = labels[1];
    while(*ptr){
    	glcd_tiny_draw_char_xy_white(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    glcd_write();
}

void drawP1Deactivate(){
	char* labels[] = {"CRUISE CONTROL", "DEACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 17;
    uint8_t x = 22;

    char* ptr = labels[0];

    while(*ptr){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    y = 40;
    x = 31;
    // play with this
    glcd_fill_rect(30, correct_Y(y-1), 68, 9, 1);

    ptr = labels[1];

    while(*ptr != '\0'){
    	glcd_tiny_draw_char_xy_white(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    glcd_write();
}

void drawP1IgnitionOff(){
	char* labels[] = {"CAR IS", "SLEEPING"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 12;
    uint8_t x = 46;

    char* ptr = labels[0];

    while(*ptr){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    y = 36;
    x = 40;

    ptr = labels[1];

    while(*ptr){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

	glcd_write();
}

void drawP1BMSFault(){
    char* labels[] = {"BMS FAULT DETECTED", "CAR", "OFF"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 5;
    uint8_t x = 10;

    char* ptr = labels[0];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    y = y+23;
    x = 43;

    ptr = labels[1];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    x = 67;
    glcd_fill_rect(x-1, correct_Y(y-1), 20, 9, 1);

    ptr = labels[2];

	while(*ptr != '\0'){
		glcd_tiny_draw_char_xy_white(x, correct_Y(y), *ptr);
		x+=6;
		ptr++;
	}

	y=y+23;

	char* faultType = "";
	uint8_t faultNum = 0;
	uint8_t faultCell = 0;
	uint8_t faultTypeL = 0;

	switch(detailed_data.faultType){
	case BATTERY_FAULT_OVERTEMPERATURE:
		faultType = "OVERTEMP";
		faultNum = 0;
		faultTypeL = 8;
		faultCell = detailed_data.faultTherm;
		break;
	case BATTERY_FAULT_UNDERTEMPERATURE:
		faultType = "UNDERTEMP";
		faultNum = 1;
		faultTypeL = 8;
		faultCell = detailed_data.faultCell;
		break;
	case BATTERY_FAULT_OVERVOLTAGE:
		faultType = "OVERVOLT";
		faultNum = 1;
		faultTypeL = 8;
		faultCell = detailed_data.faultCell;
		break;
	case BATTERY_FAULT_UNDERVOLTAGE:
		faultType = "UNDERVOLT";
		faultNum = 1;
		faultTypeL = 9;
		faultCell = detailed_data.faultCell;
		break;
	case BATTERY_FAULT_OVERCURRENT:
		faultType = "OVERCURRENT";
		faultNum = 2;
		faultTypeL = 11;
		faultCell = 0xFF;
		break;
	}

	x = 70 - faultTypeL*6;
	while(*faultType != '\0'){
		glcd_tiny_draw_char_xy(x, correct_Y(y), *faultType);
		x+=6;
		faultType++;
	}
	if(faultCell != 0xFF){
		x = 70;
		char cellString[11] = {0};
		if (faultNum == 0) sprintf(cellString, " (THERM %d)\0", faultCell);
		else if (faultNum == 1) sprintf(cellString, " (CELL %d)\0", faultCell);
		ptr = cellString;

		while(*ptr != '\0'){
			glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
			x+=6;
			ptr++;
		}
	}

	glcd_write();
}

// draw p1
void drawP1(uint8_t sel){
	pToggle = 0;
	/* Display selection (sel):
	 * 0: Default
	 * 1: Detailed
	 * 2: Cruise control activated
	 * 3: Cruise control deactivated
	 * 4: Ignition off (car is sleeping)
	 * 5: BMS fault
	 */
	drawP1Default_new();
//	switch(sel){
//	case 0:
//		drawP1Default();
//		break;
//	case 1:
//		drawP1Detailed(/*defaultDetailed*/);
//		break;
//	case 2:
//		drawP1Activate();
//		break;
//	case 3:
//		drawP1Deactivate();
//		break;
//	case 4:
//		drawP1IgnitionOff();
//		break;
//	case 5:
//		drawP1BMSFault();
//		break;
//	default:
//		drawP1Default(/*defaultTest*/);
//		break;
//	}
}

// p2 stuff start
void testAlpha(){
	glcd_clear_buffer();
	uint8_t y = 0;
	glcd_set_font(JetBrains_Mono13x20, 13, 22, 'A', 'Z');
	int offset = 0;
	for(int j = 0; j < 3; j++){
		for(int i = 0; i < 8; i++){
			glcd_draw_char_xy(0+(i*15), correct_Y(y), 'A' + offset);
			offset++;
		}
		y+=20;
	}
	glcd_write();
}
void drawP2Default(/*int value[4]*/){
	uint8_t value[4] = {default_data.P2_DRL_state,
						default_data.P2_motor_state,
						default_data.P2_VFM,
						common_data.battery_soc};

	if(value[3] > 99) value[3] = 99;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();


	glcd_draw_line(44, 0,  44, 63, BLACK);
	glcd_tiny_draw_char_xy(19, correct_Y(52), '%');

    glcd_tiny_set_font(Font5x7,5,7,32,127);

	// write the 2 on off
	uint8_t y = 3;
	glcd_set_font(JetBrains_Mono13x20, 13, 20, 'A', 'Z');

	char* state;
	uint8_t stateL;

	switch(value[1]){
	case 1:
		state = "PEDAL";
		stateL = 5;
		break;
	case 2:
		state = "CRUISE";
		stateL = 6;
		break;
	case 3:
		state = "REGEN";
		stateL = 5;
		break;
	default:
		state = "OFF";
		stateL = 3;
		break;
	}

	for(int i = 0; i < stateL; i++){
		glcd_draw_char_xy(49+i*13, 25, state[i]);
	}

	glcd_draw_char_xy(49, correct_Y(45), 'V');
	glcd_draw_char_xy(62, correct_Y(45), 'F');
	glcd_draw_char_xy(75, correct_Y(45), 'M');
	glcd_fill_rect(90, correct_Y(48), 2, 3, 1);
	glcd_fill_rect(90, correct_Y(54), 2, 3, 1);

	// write VFM stat
	glcd_set_font(JetBrains_Mono13x21_Symbol, 13, 21, ' ', '9');
	glcd_draw_char_xy(98, correct_Y(44), '0' + value[2]);

	// now write the big battery
	glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
	if((value[3]/10) != 0) glcd_draw_char_xy(0, 16, '0' + value[3]/10);
	glcd_draw_char_xy(21, 16, '0' + value[3]%10);

	glcd_write();
}

void drawP2DefaultLow(/*int value[4]*/){
	uint8_t value[2] = {default_data.P2_low_supp_volt,
						common_data.battery_soc};

	if(value[1] > 99) value[1] = 99;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 5;
	uint8_t x = 0;

	glcd_draw_line(44, 0,  44, 63, BLACK);
	glcd_tiny_draw_char_xy(19, correct_Y(52), '%');

    glcd_tiny_set_font(Font5x7,5,7,32,127);
    char* low = "LOW SUPP.";
    char* volt = "VOLTAGE";

    // draw the labels
	int j = 0;
	x = 59;
	y = 41;
	// char by char cuz draw xy only with char
	while(low[j] != '\0'){
		glcd_tiny_draw_char_xy(59+j*6, correct_Y(y), low[j]);
		j++;
	}

	y = 52;
	while(volt[j] != '\0'){
		glcd_tiny_draw_char_xy(65+j*6, correct_Y(y), volt[j]);
		j++;
	}



    // draw attention icon
    x = 44;
    y = 5;
    for(int i = 0; i < 28; i++){
		for(int j = 0; j < 32; j++){
			if(leaf[i][j])
				glcd_set_pixel(x+j, y+i, 1);
		}
	}
    x = 75;
    y = 9;
    glcd_set_font(JetBrains_Mono13x21_Symbol, 13, 21, ' ', '9');
	char valueS[4] = {0};
	sprintf(valueS, "%3d", value[0]);
	for(int i = 0; i < 3; i++){
		glcd_draw_char_xy(x+(i*13), correct_Y(y), valueS[i]);
	}
	glcd_set_font(JetBrains_Mono13x20, 13, 20, 'A', 'Z');
	glcd_draw_char_xy(115, correct_Y(y), 'V');

	// now write the big battery
	glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
	if((value[1]/10) != 0) glcd_draw_char_xy(0, 16, '0' + value[1]/10);
	glcd_draw_char_xy(21, 16, '0' + value[1]%10);

	glcd_write();
}


void drawP2Detailed_1(/*int value[9]*/){
	char labelsP1[3][25] = {0};
	int labelsP1L = 3;
	short value[6] = {	common_data.solar_power,
						common_data.motor_power,
						common_data.battery_power,
						detailed_data.P1_solar_voltage,
						detailed_data.P1_motor_voltage,
						detailed_data.P1_battery_voltage};
	float currents[3] = {
		detailed_data.P1_solar_current,
		detailed_data.P1_motor_current,
		detailed_data.P1_battery_current
	};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// populate label
	if(value[0] >= 0) sprintf(labelsP1[0], "S:+%4dW,%3dV,+%2d.%1dA\0", abs(value[0]), abs(value[3]), abs(currents[0]), abs(currents[0] * 10) % 10);
	else              sprintf(labelsP1[0], "S:-%4dW,%3dV,-%2d.%1dA\0", abs(value[0]), abs(value[3]), abs(currents[0]), abs(currents[0] * 10) % 10);

	if(value[1] >= 0) sprintf(labelsP1[1], "M:+%4dW,%3dV,+%2d.%1dA\0", abs(value[1]), abs(value[4]), abs(currents[1]), abs(currents[1] * 10) % 10);
	else              sprintf(labelsP1[1], "M:-%4dW,%3dV,-%2d.%1dA\0", abs(value[1]), abs(value[4]), abs(currents[1]), abs(currents[1] * 10) % 10);

	if(value[2] >= 0) sprintf(labelsP1[2], "B:+%4dW,%3dV,+%2d.%1dA\0", abs(value[2]), abs(value[5]), abs(currents[2]), abs(currents[2] * 10) % 10);
	else              sprintf(labelsP1[2], "B:-%4dW,%3dV,-%2d.%1dA\0", abs(value[2]), abs(value[5]), abs(currents[2]), abs(currents[2] * 10) % 10);

	// start drawing at y = 5
	uint8_t y = 5;

	// draw the labels
	for(int i = 0; i < labelsP1L; i++){
		char* label = labelsP1[i];
		int j = 0;
		// char by char cuz draw xy only with char
		while(label[j] != '\0'){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
			j++;
		}
		// go next rows, these value are just what I think will look good
		y+=23;
	}

	glcd_write();
}

void drawP2Detailed_2(/*int value[5]*/){
	short value[4] = {	common_data.LV_power,
						common_data.LV_voltage,
						detailed_data.P2_LV_current,
						detailed_data.P2_max_batt_temp};
	char* labelsP2[] = {"LV:", ""/*, "VFM:"*/};
	int labelsP2L = 2;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 5;
	for(int i = 0; i < labelsP2L; i++){
		char* label = labelsP2[i];
		int j = 0;
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(j*(5), correct_Y(y), label[j]);
			j++;
		}
		y+=23;
	}

    // draw the numbers
	char valueS[4][4];

	// get it in strings
	for(int i = 0; i < 4; i++){
		// sign
		short v = value[i];
		if(v<0){
			valueS[i][0] = '-';
			v *= -1;
		}
		else{
			valueS[i][0] = '+';
		}
		// tenth
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// ones
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = '0';
		}
		// decimal
		valueS[i][3] = '0' + v%10;
	}

    // write the 4 small values
	y = 5;
    // the watt and V and A
	uint8_t x = 20;
	glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[0][0]);
	glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[0][1]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[0][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), '.');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[0][3]);
    glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'W');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), '(');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[1][1]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[1][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), '.');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[1][3]);
    glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'V');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), ',');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[2][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), '.');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[2][3]);
    glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'A');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), ')');
    // the temp
    y+=23;
    x = 70;
//    glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[3][0]);
//    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[3][1]);
//    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[3][2]);
//    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), '.');
//    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[3][3]);
//    glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'C');
    // VFM
    y+=23;
//    x = 20;
//    glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[4][2]);
//    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[4][3]);
    if(detailed_data.P2_BB){
    	x = 8;
    	glcd_fill_rect(x, correct_Y(y-1), 13, 9, 1);
    	glcd_tiny_draw_char_xy_white(x+=1, correct_Y(y), 'B');
    	glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'B');
    }
    if(detailed_data.P2_MC){
    	x = 28;
		glcd_fill_rect(x, correct_Y(y-1), 13, 9, 1);
		glcd_tiny_draw_char_xy_white(x+=1, correct_Y(y), 'M');
		glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'C');
	}
    if(detailed_data.P2_BMS){
    	x = 48;
		glcd_fill_rect(x, correct_Y(y-1), 19, 9, 1);
		glcd_tiny_draw_char_xy_white(x+=1, correct_Y(y), 'B');
		glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'M');
		glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'S');
	}
    if(detailed_data.P2_PPT){
    	x = 74;
		glcd_fill_rect(x, correct_Y(y-1), 19, 9, 1);
		glcd_tiny_draw_char_xy_white(x+=1, correct_Y(y), 'P');
		glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'P');
		glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'T');
	}
    if(detailed_data.P2_RAD){
    	x = 100;
		glcd_fill_rect(x, correct_Y(y-1), 19, 9, 1);
		glcd_tiny_draw_char_xy_white(x+=1, correct_Y(y), 'R');
		glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'A');
		glcd_tiny_draw_char_xy_white(x+=6, correct_Y(y), 'D');
	}

	glcd_write();
}


void drawP2Detailed_3(){
	char labels[3][50];

	sprintf(
		labels[0],
		"T:%c%2d.%d(%2d)%c%2d.%d(%2d)\0",
		detailed_data.min_temperature < 0 ? '-' : '+',
		abs(detailed_data.min_temperature) / 10,
		abs(detailed_data.min_temperature) % 10,
		detailed_data.min_temperature_cell,
		detailed_data.max_temperature < 0 ? '-' : '+',
		abs(detailed_data.max_temperature) / 10,
		abs(detailed_data.max_temperature) % 10,
		detailed_data.max_temperature_cell
	);
	sprintf(
		labels[1],
		"V:%c%2d.%d(%2d)%c%2d.%d(%2d)\0",
		detailed_data.min_voltage < 0 ? '-' : '+',
		abs(detailed_data.min_voltage) / 10,
		abs(detailed_data.min_voltage) % 10,
		detailed_data.min_voltage_cell,
		detailed_data.max_voltage < 0 ? '-' : '+',
		abs(detailed_data.max_voltage) / 10,
		abs(detailed_data.max_voltage) % 10,
		detailed_data.max_voltage_cell
	);
	sprintf(
		labels[2],
		"S:%c%2d%%(%2d)%c%2d%%(%2d)\0",
		detailed_data.min_soc < 0 ? '-' : '+',
		abs(detailed_data.min_soc),
		detailed_data.min_soc_cell,
		detailed_data.max_soc < 0 ? '-' : '+',
		abs(detailed_data.max_soc),
		detailed_data.max_soc_cell
	);

	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
	glcd_clear_buffer();

	for (uint32_t i = 0, y = 5; i < 3; ++i, y += 23)
		for (uint32_t j = 0; labels[i][j]; ++j)
			glcd_tiny_draw_char_xy(
				j * 6,
				correct_Y(y),
				labels[i][j]
			);

	glcd_write();
}

void drawP2Activate(){
	char* labels[] = {"CRUISE CONTROL", "ACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 17;
	uint8_t x = 22;

	char* ptr = labels[0];
	while(*ptr){
		glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
		x+=6;
		ptr++;
	}

	y = 40;
	x = 37;
	// play with this
	glcd_fill_rect(30, correct_Y(y-1), 68, 9, 1);

	ptr = labels[1];
	while(*ptr){
		glcd_tiny_draw_char_xy_white(x, correct_Y(y), *ptr);
		x+=6;
		ptr++;
	}

	glcd_write();
}

void drawP2Deactivate(){
	char* labels[] = {"CRUISE CONTROL", "DEACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 17;
	uint8_t x = 22;

	char* ptr = labels[0];

	while(*ptr){
		glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
		x+=6;
		ptr++;
	}

	y = 40;
	x = 31;
	// play with this
	glcd_fill_rect(30, correct_Y(y-1), 68, 9, 1);

	ptr = labels[1];

	while(*ptr != '\0'){
		glcd_tiny_draw_char_xy_white(x, correct_Y(y), *ptr);
		x+=6;
		ptr++;
	}

	glcd_write();
}

void drawP2IgnitionOff(/*int value[4]*/){
	short value[4] = {	detailed_data.P2_HV_voltage,
						common_data.LV_voltage,
						common_data.LV_power,
						(short)(common_data.battery_soc)};
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 5;

	// sleeping
	char* sleep = "SLEEPING";
	int j = 0;
	while(sleep[j] != 0){
		glcd_tiny_draw_char_xy(75+j*6, correct_Y(5), sleep[j]);
		j++;
	}

	char valueS[3][20] = {0};

	// populate label
	if(value[0] >= 0) sprintf(valueS[0], "HV:    +%3dV\0", value[0]);
	else sprintf(valueS[0], "HV:    -%3dV\0", abs(value[0]));

	uint8_t lv_before = (int)value[1];
	uint8_t lv_after = ((int)(value[1]*10))%10;
	if(value[1] >= 0) sprintf(valueS[1], "LV:   +%2d.%1dV(%2dW)\0", lv_before, lv_after, value[2]);
	else sprintf(valueS[1], "LV:   -%2d.%1dV(%2dW)\0", lv_before, lv_after, value[2]);

	sprintf(valueS[2],"Battery:%3d%%\0", value[3]);

	y = 5;
	for(int i = 0; i < 3; i++){
		char* ptr = valueS[i];
		uint8_t x = 0;
		while(*ptr){
			glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
			x+=6;
			ptr++;
		}
		y+=23;
	}

	glcd_write();
}

void drawP2BMSFault(/*int value[4]*/){
	char *labels[] = {
		"BMS OV: ",
		"BMS UV: ",
		"BMS OT: ",
		"BMS UT: ",
		"BMS OC: "
	};
	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
	glcd_clear_buffer();

	uint8_t x = 2, xStep = 6, y = 5, yStep = 10;
	char *label = labels[0];

	for (int i = 0; i < 8; ++i, x += xStep)
		glcd_tiny_draw_char_xy(x, correct_Y(y), label[i]);

	for (int i = 0; i < NUM_BATT_CELLS; ++i) {
		if (detailed_data.overvoltage_status & (1 << i)) {
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i / 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i % 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), ' ');
			x += xStep;
		}
	}

	x = 2;
	y += yStep;
	label = labels[1];

	for (int i = 0; i < 8; ++i, x += xStep)
		glcd_tiny_draw_char_xy(x, correct_Y(y), label[i]);

	for (int i = 0; i < NUM_BATT_CELLS; ++i) {
		if (detailed_data.undervoltage_status & (1 << i)) {
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i / 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i % 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), ' ');
			x += xStep;
		}
	}

	x = 2;
	y += yStep;
	label = labels[2];

	for (int i = 0; i < 8; ++i, x += xStep)
		glcd_tiny_draw_char_xy(x, correct_Y(y), label[i]);

	for (int i = 0; i < NUM_BATT_TEMP_SENSORS; ++i) {
		if (detailed_data.overtemperature_status & (1 << i)) {
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i / 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i % 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), ' ');
			x += xStep;
		}
	}

	x = 2;
	y += yStep;
	label = labels[3];

	for (int i = 0; i < 8; ++i, x += xStep)
		glcd_tiny_draw_char_xy(x, correct_Y(y), label[i]);

	for (int i = 0; i < NUM_BATT_TEMP_SENSORS; ++i) {
		if (detailed_data.undertemperature_status & (1 << i)) {
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i / 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), '0' + (i % 10));
			x += xStep;
			glcd_tiny_draw_char_xy(x, correct_Y(y), ' ');
			x += xStep;
		}
	}

	x = 2;
	y += yStep;
	label = labels[4];

	for (int i = 0; i < 8; ++i, x += xStep)
		glcd_tiny_draw_char_xy(x, correct_Y(y), label[i]);

	if (detailed_data.overcurrent_status)
		glcd_tiny_draw_char_xy(x, correct_Y(y), '1');
	else
		glcd_tiny_draw_char_xy(x, correct_Y(y), '0');

	glcd_write();
}

void drawP2(uint8_t sel){
	pToggle = 1;

	switch(sel){
	case 0:
		drawP2Default(/*defaultTest*/);
		break;
	case 1:
		drawP2Detailed_1(/*defaultDetailed*/);
		break;
	case 2:
		drawP2Detailed_2(/*defaultDetailed*/);
		break;
	case 3:
		drawP2Detailed_3();
		break;
	case 4:
		drawP2Activate();
		break;
	case 5:
		drawP2Deactivate();
		break;
	case 6:
		drawP2IgnitionOff(/*defaultBMSFault*/);
		break;
	case 7:
		drawP2BMSFault(/*defaultBMSFault*/);
		break;
	case 8:
		drawP2DefaultLow();
		break;
	default:
		drawP2Default(/*defaultTest*/);
		break;
	}
}
