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

/** Fonts */
#include "fonts/font5x7.h"
#include "fonts/Liberation_Sans11x14_Numbers.h"
#include "fonts/Liberation_Sans15x21_Numbers.h"
#include "fonts/Liberation_Sans17x17_Alpha.h"
#include "fonts/Liberation_Sans27x36_Numbers.h"
#include "fonts/Liberation_Sans20x28_Numbers.h"
#include "fonts/Bebas_Neue20x36_Bold_Numbers.h"
//#include "fonts/Earthbound_12x19_48to57.h"
#include "fonts/font5x7.h"

uint8_t correct_Y(uint8_t y){
	if(pToggle)
		return y;
	return (y+16)%64;
}

// p1 stuff start

void drawP1Default(/*int value[4]*/){
	short value[4] = {	common_data->solar_power,
						common_data->motor_power,
						common_data->battery_power,
						(short)(default_data->P1_speed_kph)};

	char* labelsP1[] = {"Solar:", "Motor:", "Battery:"};
	int labelsP1L = 3;
	char* labelspeed = "km/h";

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 5;

	// draw the labels
	for(int i = 0; i < labelsP1L; i++){
		char* label = labelsP1[i];
		int j = 0;
		// char by char cuz draw xy only with char
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
			j++;
		}
		glcd_tiny_draw_char_xy(72, correct_Y(y), 'w');
		// go next rows, these value are just what I think will look good
		y+=23;
	}

	// draw divider line
	glcd_draw_line(81, 0,  81, 63, BLACK);
	int x = 0;
	// draw km/h
	while(labelspeed[x] != 0){
		glcd_tiny_draw_char_xy(94+(x*6), correct_Y(52), labelspeed[x]);
		x++;
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
		// hundred
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// tenth
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = ' ';
		}
		// ones
		valueS[i][3] = '0' + v%10;
	}

	// write the 3 small values
	y = 5;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			glcd_tiny_draw_char_xy(48+(j*6), correct_Y(y), valueS[i][j]);
		}
		y+=23;
	}

	// now write the big speed
	glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
	glcd_draw_char_xy(85, correct_Y(16), valueS[3][2]);
	glcd_draw_char_xy(105, correct_Y(16), valueS[3][3]);

	glcd_write();
}

void drawP1Detailed(/*int value[9]*/){
	char* labelsP1[] = {"Solar:", "Motor:", "Battery:"};
	int labelsP1L = 3;
	short value[9] = {	common_data->solar_power,
						common_data->motor_power,
						common_data->battery_power,
						detailed_data->P1_solar_voltage, detailed_data->P1_solar_current,
						detailed_data->P1_motor_voltage, detailed_data->P1_motor_current,
						detailed_data->P1_battery_voltage, detailed_data->P1_battery_current};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 5;

	// draw the labels
	for(int i = 0; i < labelsP1L; i++){
		char* label = labelsP1[i];
		int j = 0;
		// char by char cuz draw xy only with char
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
			j++;
		}
		glcd_tiny_draw_char_xy(68, correct_Y(y), 'w');
		// go next rows, these value are just what I think will look good
		y+=23;
	}

	// draw the numbers
	char valueS[9][4];

	glcd_tiny_set_font(Font5x7,5,7,32,127);

	// get it in strings
	for(int i = 0; i < 9; i++){
		// sign
		short v = value[i];
		if(v<0){
			valueS[i][0] = '-';
			v *= -1;
		}
		else{
			valueS[i][0] = '+';
		}
		// hundred
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// tenth
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = ' ';
		}
		// ones
		valueS[i][3] = '0' + v%10;
	}

	// write the 3 small values
	y = 5;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			glcd_tiny_draw_char_xy(48+(j*5), correct_Y(y), valueS[i][j]);
		}
		y+=23;
	}

	// write the 6 small values
	y = 5;
	for(int i = 3; i < 9; i+=2){
		uint8_t x = 73;
        glcd_tiny_draw_char_xy(x, correct_Y(y), '(');
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[i][1]);
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[i][2]);
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[i][3]);
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), 'V');
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), ',');
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[i+1][2]);
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[i+1][3]);
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), 'A');
        glcd_tiny_draw_char_xy(x+=5, correct_Y(y), ')');
		y+=23;
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
	uint8_t y = 12;
    uint8_t x = 10;

    char* ptr = labels[0];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
        ptr++;
    }

    y = 36;
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

	glcd_write();
}

// draw p1
void drawP1(uint8_t sel){
	pToggle = 0;
//	int defaultTest[4] = {420, 874, -454, 69};
//	int defaultDetailed[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

	switch(sel){
	case 0:
		drawP1Default(/*defaultTest*/);
		break;
	case 1:
		drawP1Detailed(/*defaultDetailed*/);
		break;
	case 2:
		drawP1Activate();
		break;
	case 3:
		drawP1Deactivate();
		break;
	case 4:
		drawP1IgnitionOff();
		break;
	case 5:
		drawP1BMSFault();
		break;
	default:
		drawP1Default(/*defaultTest*/);
		break;
	}

	glcd_bbox_refresh();
	pToggle = 1;
	glcd_write();
	pToggle = 0;
}

// p2 stuff start
void drawP2Default(/*int value[4]*/){
	uint8_t value[4] = {default_data->P2_cruise_state,
						default_data->P2_DRL_state,
						default_data->P2_regen_state,
						common_data->battery_soc};
	char* labelsP2[] = {"Cruise:", "DRL:"};
	int labelsP2L = 2;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 5;
	for(int i = 0; i < labelsP2L; i++){
		char* label = labelsP2[i];
		int j = 0;
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(50+j*6, correct_Y(y), label[j]);
			j++;
		}
		y+=23;
	}

	glcd_draw_line(44, 0,  44, 63, BLACK);
	glcd_tiny_draw_char_xy(19, correct_Y(52), '%');

    glcd_tiny_set_font(Font5x7,5,7,32,127);

	// write the 2 on off
	y = 5;
	for(int i = 0; i < 2; i++){
		char* status = "OFF";
		if(value[i]) status = " ON";
		for(int j = 0; j < 3; j++){
			glcd_tiny_draw_char_xy(95+(j*6), correct_Y(y), status[j]);
		}
		y+=23;
	}

	char* regen = "     ";
	if(value[2]) regen = "REGEN";

	glcd_set_font(Liberation_Sans17x17_Alpha, 17, 17, 'A', 'Z');
	for(int i = 0; i < 5; i++){
		glcd_draw_char_xy(49+i*15, 45, regen[i]);
	}

	// now write the big battery
	glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
	glcd_draw_char_xy(0, 16, '0' + value[3]/10);
	glcd_draw_char_xy(21, 16, '0' + value[3]%10);

	glcd_write();
}

void drawP2Detailed(/*int value[5]*/){
	short value[5] = {	common_data->LV_power,
						common_data->LV_voltage,
						detailed_data->P2_LV_current,
						detailed_data->P2_max_batt_temp,
						detailed_data->P2_VFM_state};
	char* labelsP2[] = {"Lo Voltage:", "Max pack temp:", "VFM:"};
	int labelsP2L = 3;

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
	char valueS[5][4];

	// get it in strings
	for(int i = 0; i < 5; i++){
		// sign
		short v = value[i];
		if(v<0){
			valueS[i][0] = '-';
			v *= -1;
		}
		else{
			valueS[i][0] = '+';
		}
		// hundred
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// tenth
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = ' ';
		}
		// ones
		valueS[i][3] = '0' + v%10;
	}

    // write the 5 small values
	y = 5;
    // the watt and V and A
	uint8_t x = 55;
	glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[0][0]);
	glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[0][1]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[0][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[0][3]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), 'W');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), '(');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[1][1]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[1][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[1][3]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), 'V');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), ',');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[2][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[2][3]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), 'A');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), ')');
    // the temp
    y+=23;
    x = 70;
    glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[3][0]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[3][1]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[3][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), '.');
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[3][3]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), 'C');
    // VFM
    y+=23;
    x = 20;
    glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[4][2]);
    glcd_tiny_draw_char_xy(x+=5, correct_Y(y), valueS[4][3]);

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
	short value[4] = {	detailed_data->P2_HV_voltage,
						common_data->LV_voltage,
						common_data->LV_power,
						(short)(common_data->battery_soc)};
	char* labelsP2[] = {"HV:", "LV:", "Battery:"};
	int labelsP2L = 3;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 5;
	for(int i = 0; i < labelsP2L; i++){
		char* label = labelsP2[i];
		int j = 0;
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
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
		// hundred
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// tenth
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = ' ';
		}
		// ones
		valueS[i][3] = '0' + v%10;
	}

	// write the 4 small values
	y = 5;
	for(int i = 0; i < 3; i++){
		uint8_t x = 48;
        switch(i){
            case 0:
                glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[0][0]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[0][1]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[0][2]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[0][3]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'V');
                break;
            case 1:
                glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[1][0]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[1][1]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[1][2]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), '.');
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[1][3]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'V');
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), '(');
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[2][2]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[2][3]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'W');
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), ')');
                break;
            case 2:
            	glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[3][1]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[3][2]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[3][3]);
                glcd_tiny_draw_char_xy(x+=6, correct_Y(y), '%');
                break;
            default:
                break;
        }
		y+=23;
	}

	glcd_write();
}

void drawP2BMSFault(/*int value[4]*/){
	short value[4] = {	detailed_data->P2_HV_voltage,
						common_data->LV_voltage,
						common_data->LV_power,
						(short)(common_data->battery_soc)};
	char* labelsP2[] = {"HV:", "LV:", "Battery:"};
	int labelsP2L = 3;

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	uint8_t y = 5;
	for(int i = 0; i < labelsP2L; i++){
		char* label = labelsP2[i];
		int j = 0;
		while(label[j] != 0){
			glcd_tiny_draw_char_xy(j*6, correct_Y(y), label[j]);
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
		// hundred
		if(v/100 != 0){
			valueS[i][1] = '0' + v/100;
		}
		else{
			valueS[i][1] = ' ';
		}
		// tenth
		if((v/10)%10 != 0 || valueS[i][1] != ' '){
			valueS[i][2] = '0' + (v/10)%10;
		}
		else{
			valueS[i][2] = ' ';
		}
		// ones
		valueS[i][3] = '0' + v%10;
	}

	// write the 4 small values
	y = 5;
	for(int i = 0; i < 3; i++){
		uint8_t x = 48;
		switch(i){
			case 0:
				glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[0][0]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[0][1]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[0][2]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[0][3]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'V');
				break;
			case 1:
				glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[1][0]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[1][1]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[1][2]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), '.');
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[1][3]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'V');
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), '(');
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[2][2]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[2][3]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), 'W');
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), ')');
				break;
			case 2:
				glcd_tiny_draw_char_xy(x, correct_Y(y), valueS[3][1]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[3][2]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), valueS[3][3]);
				glcd_tiny_draw_char_xy(x+=6, correct_Y(y), '%');
				break;
			default:
				break;
		}
		y+=23;
	}

	glcd_write();
}

void drawP2(uint8_t sel){
	pToggle = 1;
//	int defaultTest[4] = {1, 0, 1, 87};
//	int defaultDetailed[5] = {1, 2, 3, 4, 5};
//	int defaultBMSFault[4] = {89, 132, 13, 49};

	switch(sel){
	case 0:
		drawP2Default(/*defaultTest*/);
		break;
	case 1:
		drawP2Detailed(/*defaultDetailed*/);
		break;
	case 2:
		drawP2Activate();
		break;
	case 3:
		drawP2Deactivate();
		break;
	case 4:
		drawP2IgnitionOff(/*defaultBMSFault*/);
		break;
	case 5:
		drawP2BMSFault(/*defaultBMSFault*/);
		break;
	default:
		drawP2Default(/*defaultTest*/);
		break;
	}
}
