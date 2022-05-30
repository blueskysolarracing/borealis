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
int correct_Y(uint8_t y){
	return ((y+48)%64);
}

// p1 stuff start

void drawP1Default(int value[4]){
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
		int v = value[i];
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
	glcd_draw_char_xy(85, 0, valueS[3][2]);
	glcd_draw_char_xy(105, 0, valueS[3][3]);

	uint8_t datajunk[3] = {1, 2, 7};
	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISP_RST_1_GPIO_Port, DISP_RST_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISP_LED_CTRL_GPIO_Port, DISP_LED_CTRL_Pin, GPIO_PIN_SET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_SET);
}

void drawP1Detailed(int value[9]){
	char* labelsP1[] = {"Solar:", "Motor:", "Battery:"};
	int labelsP1L = 3;

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

	// draw the numbers
	char valueS[9][4];

	glcd_tiny_set_font(Font5x7,5,7,32,127);

	// get it in strings
	for(int i = 0; i < 9; i++){
		// sign
		int v = value[i];
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

	// write the 6 small values
	y = 5;
	for(int i = 3; i < 9; i+=2){
        glcd_tiny_draw_char_xy(78, correct_Y(y), '(');
        glcd_tiny_draw_char_xy(82, correct_Y(y), valueS[i][2]);
        glcd_tiny_draw_char_xy(87, correct_Y(y), valueS[i][3]);
        glcd_tiny_draw_char_xy(92, correct_Y(y), 'V');
        glcd_tiny_draw_char_xy(97, correct_Y(y), ',');
        glcd_tiny_draw_char_xy(102, correct_Y(y), valueS[i+1][2]);
        glcd_tiny_draw_char_xy(107, correct_Y(y), valueS[i+1][3]);
        glcd_tiny_draw_char_xy(112, correct_Y(y), 'A');
        glcd_tiny_draw_char_xy(117, correct_Y(y), ')');
		y+=23;
	}

	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_SET);
}

void drawP1Activate(){
	char* labels[] = {"CRUISE CONTROL", "ACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 12;
    uint8_t x = 22;

    char* ptr = labels[0];
    int cnt = 0;
    while(*ptr != 0){
    	if (ptr[cnt] == 0){ break; }
        glcd_tiny_draw_char_xy(x, correct_Y(y), ptr[cnt]);
        cnt++;
        x+=6;
    }

    y = 36;
    x = 37;

    ptr = labels[1];
    cnt = 0;
    while(*ptr != 0){
    	if (ptr[cnt] == 0){ break; }
        glcd_tiny_draw_char_xy(x, correct_Y(y), ptr[cnt]);
        cnt++;
        x+=6;
    }

	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_SET);
}

void drawP1Deactivate(){
	char* labels[] = {"CRUISE CONTROL", "DEACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 12;
    uint8_t x = 22;

    char* ptr = labels[0];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

    y = 36;
    x = 31;

    ptr = labels[1];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_SET);
}

void drawP1IgnitionOff(){
    char* labels[] = {"CAR IS", "SLEEPING"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 12;
    uint8_t x = 46;

    char* ptr = labels[0];
    int cnt = 0;

    while(ptr[cnt] != NULL){
        glcd_tiny_draw_char_xy(x, correct_Y(y), ptr[cnt]);
        x+=6;
        cnt++;
    }

    y = 36;
    x = 40;

    ptr = labels[1];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_SET);
}

void drawP1BMSFault(){
    char* labels[] = {"BMS FAULT DETECTED", "CAR OFF"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 12;
    uint8_t x = 10;

    char* ptr = labels[0];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

    y = 36;
    x = 43;

    ptr = labels[1];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_0_GPIO_Port, DISP_CS_0_Pin, GPIO_PIN_SET);
}

// draw p1
void drawP1(){
	int defaultTest[4] = {420, 874, -454, 69};
	int defaultDetailed[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	drawP1Default(defaultTest);
//	drawP1Detailed(defaultDetailed);
//	drawP1Activate();
//	drawP1Deactivate();
//	drawP1IgnitionOff();
//	drawP1BMSFault();
}

// p2 stuff start
void drawP2Default(int value[4]){
	char* labelsP2[] = {"Cruise:", "Light:"};
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
		glcd_draw_char_xy(49+i*15, 30, regen[i]);
	}

	// now write the big battery
	glcd_set_font(Liberation_Sans20x28_Numbers, 20, 28, '.', '9');
	glcd_draw_char_xy(0, 0, '0' + value[3]/10);
	glcd_draw_char_xy(21, 0, '0' + value[3]%10);

	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_SET);
}

void drawP2Detailed(int value[4]){
	char* labelsP2[] = {"Low voltage:", "Max pack temp:"};
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
		int v = value[i];
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
    // the watt and V and A
    glcd_tiny_draw_char_xy(60, correct_Y(y), valueS[0][2]);
    glcd_tiny_draw_char_xy(65, correct_Y(y), valueS[0][3]);
    glcd_tiny_draw_char_xy(70, correct_Y(y), 'W');
    glcd_tiny_draw_char_xy(75, correct_Y(y), '(');
    glcd_tiny_draw_char_xy(80, correct_Y(y), valueS[1][2]);
    glcd_tiny_draw_char_xy(85, correct_Y(y), valueS[1][3]);
    glcd_tiny_draw_char_xy(90, correct_Y(y), 'V');
    glcd_tiny_draw_char_xy(95, correct_Y(y), ',');
    glcd_tiny_draw_char_xy(100, correct_Y(y), valueS[2][2]);
    glcd_tiny_draw_char_xy(105, correct_Y(y), valueS[2][3]);
    glcd_tiny_draw_char_xy(110, correct_Y(y), 'A');
    glcd_tiny_draw_char_xy(115, correct_Y(y), ')');
    // the temp
    y+=23;
    glcd_tiny_draw_char_xy(70, correct_Y(y), valueS[3][0]);
    glcd_tiny_draw_char_xy(75, correct_Y(y), valueS[3][1]);
    glcd_tiny_draw_char_xy(80, correct_Y(y), valueS[3][2]);
    glcd_tiny_draw_char_xy(85, correct_Y(y), valueS[3][3]);
    glcd_tiny_draw_char_xy(90, correct_Y(y), 'C');

	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_SET);
}

void drawP2Activate(){
	char* labels[] = {"CRUISE CONTROL", "ACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 12;
    uint8_t x = 22;

    char* ptr = labels[0];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

    y = 36;
    x = 37;

    ptr = labels[1];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_SET);
}

void drawP2Deactivate(){
	char* labels[] = {"CRUISE CONTROL", "DEACTIVATED"};

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	// start drawing at y = 5
	uint8_t y = 12;
    uint8_t x = 22;

    char* ptr = labels[0];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

    y = 36;
    x = 31;

    ptr = labels[1];

    while(*ptr != '\0'){
        glcd_tiny_draw_char_xy(x, correct_Y(y), *ptr);
        x+=6;
    }

	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_SET);
}

void drawP2IgnitionOff(int value[4]){
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
		int v = value[i];
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
        switch(i){
            case 0:
                glcd_tiny_draw_char_xy(48, correct_Y(y), valueS[0][2]);
                glcd_tiny_draw_char_xy(54, correct_Y(y), valueS[0][3]);
                glcd_tiny_draw_char_xy(60, correct_Y(y), 'V');
                break;
            case 1:
                glcd_tiny_draw_char_xy(48, correct_Y(y), valueS[1][2]);
                glcd_tiny_draw_char_xy(54, correct_Y(y), valueS[1][3]);
                glcd_tiny_draw_char_xy(60, correct_Y(y), 'V');
                glcd_tiny_draw_char_xy(66, correct_Y(y), '(');
                glcd_tiny_draw_char_xy(72, correct_Y(y), valueS[2][2]);
                glcd_tiny_draw_char_xy(78, correct_Y(y), valueS[2][3]);
                glcd_tiny_draw_char_xy(84, correct_Y(y), 'W');
                glcd_tiny_draw_char_xy(90, correct_Y(y), ')');
                break;
            case 2:
                glcd_tiny_draw_char_xy(48, correct_Y(y), valueS[3][2]);
                glcd_tiny_draw_char_xy(54, correct_Y(y), valueS[3][3]);
                glcd_tiny_draw_char_xy(60, correct_Y(y), '%');
                break;
            default:
                break;
        }
		for(int j = 0; j < 4; j++){
			glcd_tiny_draw_char_xy(48+(j*6), correct_Y(y), valueS[i][j]);
		}
		y+=23;
	}

	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_SET);
}

void drawP2BMSFault(int value[4]){
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
		int v = value[i];
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
        switch(i){
            case 0:
                glcd_tiny_draw_char_xy(48, correct_Y(y), valueS[0][2]);
                glcd_tiny_draw_char_xy(54, correct_Y(y), valueS[0][3]);
                glcd_tiny_draw_char_xy(60, correct_Y(y), 'V');
                break;
            case 1:
                glcd_tiny_draw_char_xy(48, correct_Y(y), valueS[1][2]);
                glcd_tiny_draw_char_xy(54, correct_Y(y), valueS[1][3]);
                glcd_tiny_draw_char_xy(60, correct_Y(y), 'V');
                glcd_tiny_draw_char_xy(66, correct_Y(y), '(');
                glcd_tiny_draw_char_xy(72, correct_Y(y), valueS[2][2]);
                glcd_tiny_draw_char_xy(78, correct_Y(y), valueS[2][3]);
                glcd_tiny_draw_char_xy(84, correct_Y(y), 'W');
                glcd_tiny_draw_char_xy(90, correct_Y(y), ')');
                break;
            case 2:
                glcd_tiny_draw_char_xy(48, correct_Y(y), valueS[3][2]);
                glcd_tiny_draw_char_xy(54, correct_Y(y), valueS[3][3]);
                glcd_tiny_draw_char_xy(60, correct_Y(y), '%');
                break;
            default:
                break;
        }
		for(int j = 0; j < 4; j++){
			glcd_tiny_draw_char_xy(48+(j*6), correct_Y(y), valueS[i][j]);
		}
		y+=23;
	}

	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_RESET);
	glcd_write();
	HAL_GPIO_WritePin(DISP_CS_1_GPIO_Port, DISP_CS_1_Pin, GPIO_PIN_SET);
}

void drawP2(){
	int defaultTest[4] = {1, 0, 1, 87};
	int defaultDetailed[4] = {1, 2, 3, 4};
	int defaultBMSFault[4] = {89, 13, 13, 49};
	drawP2Default(defaultTest);
//	drawP2Detailed(defaultDetailed);
//	drawP2Activate();
//	drawP2Deactivate();
//	drawP2IgnitionOff(defaultBMSFault);
//	drawP2BMSFault(defaultBMSFault);
}

void selectP1(){
//	#define CONTROLLER_SPI_SS_PORT  DISP_CS_0_GPIO_Port
//	#define CONTROLLER_SPI_SS_PIN   DISP_CS_0_Pin
//	#define CONTROLLER_SPI_SS_RCC	RCC_AHB4Periph_GPIOI
//	#define CONTROLLER_SPI_RST_PORT DISP_RST_1_GPIO_Port
//	#define CONTROLLER_SPI_RST_PIN  DISP_RST_1_Pin
//	#define CONTROLLER_SPI_RST_RCC	RCC_AHB4Periph_GPIOJ
}

void selectP2(){
	#define CONTROLLER_SPI_SS_PORT  DISP_CS_1_GPIO_Port
	#define CONTROLLER_SPI_SS_PIN   DISP_CS_1_Pin
	#define CONTROLLER_SPI_SS_RCC	RCC_AHB4Periph_GPIOE
//	#define CONTROLLER_SPI_DC_PORT  DISP_A0_GPIO_Port
//	#define CONTROLLER_SPI_DC_PIN   DISP_A0_Pin
//	#define CONTROLLER_SPI_DC_RCC	RCC_AHB4Periph_GPIOJ
	#define CONTROLLER_SPI_RST_PORT DISP_RST_2_GPIO_Port
	#define CONTROLLER_SPI_RST_PIN  DISP_RST_2_Pin
	#define CONTROLLER_SPI_RST_RCC	RCC_AHB4Periph_GPIOJ
}
