/**
 * \file unit_tests.c
 * \brief Various test functions to demonstrate features of the library
 * \author Andy Gock
 */

/*
	Copyright (c) 2012, Andy Gock

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		  notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.
		* Neither the name of Andy Gock nor the
		  names of its contributors may be used to endorse or promote products
		  derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL ANDY GOCK BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include "glcd.h"
#include "unit_tests.h"

/** Fonts */
#include "fonts/font5x7.h"
#include "fonts/Liberation_Sans11x14_Numbers.h"
#include "fonts/Liberation_Sans15x21_Numbers.h"
#include "fonts/Liberation_Sans17x17_Alpha.h"
#include "fonts/Liberation_Sans27x36_Numbers.h"
#include "fonts/Bebas_Neue20x36_Bold_Numbers.h"
volatile uint8_t unit_test_return = 0;

/**
 *  Return from test procedure. Global var \p unit_test_return is set to 1 elsewhere,
 *  to signal function to return.
 */
#define DEMO_RETURN() if (unit_test_return) { unit_test_return = 0; return; }

#if defined(GLCD_UNIT_TEST_BITMAP_ENABLE)
	/* Open Source logo -- Size: 128x64 */
	#if defined(GLCD_DEVICE_AVR8)
		const unsigned char bmp_oslogo[] PROGMEM = {
	#else
		const unsigned char bmp_oslogo[] = {	
	#endif /* GLCD_DEVICE_AVR8 */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xc0, 0xc0, 0xc0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xc0, 0xc0, 0xc0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xfc, 0xf8, 0xf0, 0xe0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x07, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf0, 0xe0, 0xc0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xc0, 0xf0, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x0f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x1f, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x0f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // 
	};
#endif /* GLCD_UNIT_TEST_BITMAP_ENABLE */

void glcd_test_circles(void)
{
	uint8_t x,y,radius;

	while (1) {
		uint8_t i;
		
		glcd_clear();

		// generate random(ish) position on display
		x = rand() % GLCD_LCD_WIDTH;
		y = rand() % GLCD_LCD_HEIGHT;
		radius = rand() % 50;

		//x = 70; y=25; radius=50; // for debugging

		// fill circle with black
		
		for (i=0; i<=radius; i++) {
			glcd_fill_circle(x,y,i,BLACK);
			glcd_write();
//			delay_ms(2);
		}

		// fill the same circle above but with white
		for (i=0; i<=radius; i++) {
			glcd_fill_circle(x,y,i,WHITE);
			glcd_write();
//			delay_ms(1);
		}
		DEMO_RETURN();
	}
}

void glcd_test_counter_and_graph(void)
{
	uint8_t count = 0;
	char string[8] = "";
	while(1) {
		glcd_clear_buffer();

		//glcd_tiny_set_font(Font5x7,5,7,32,127);
		//glcd_draw_string_xy(0,40,(char *)utoa(count,string,10));

		glcd_set_font(Liberation_Sans15x21_Numbers,15,21,46,57);
		//glcd_set_font(Liberation_Sans27x36_Numbers,27,36,46,57);
		//glcd_set_font(Bebas_Neue20x36_Bold_Numbers,20,36,46,57);
		//glcd_set_font(Bebas_Neue18x36_Numbers,18,36,46,57);
		//glcd_set_font(HelveticaNeueLT_Com_57_Cn23x35_Numbers,23,35,46,57); // commercial font - not for public distribution

		sprintf(string,"%d",count);
		glcd_draw_string_xy(0,0,string);
		glcd_bar_graph_horizontal(10,38,30,6,count*4);
		glcd_bar_graph_vertical(70,0,8,30,count*2);

		glcd_write();
		count += 1;

		DEMO_RETURN();
	}

}

/* Test glcd-utils font table type, increments and display a 16-bit number over and over */
#include "fonts/Earthbound_12x19_48to57.h"
void glcd_test_glcdutils(void)
{
	uint16_t count = 0;
	char string[8] = "";
	while(1) {
		glcd_clear_buffer();

		/* Set the font */
		glcd_font(font_Earthbound_12x19_48to57,12,19,48,57,GLCD_UTILS);

		sprintf(string,"%d",count);
		glcd_draw_string_xy(0,0,string);
		glcd_write();
		
		count += 1;

		DEMO_RETURN();
	}
}

void glcd_test_text_up_down(void)
{
	// moves some text up and down the display

	uint8_t y;
	uint8_t max_y;
	
	//glcd_set_font(Liberation_Sans11x14_Numbers,11,14,46,57);
	glcd_set_font(Liberation_Sans15x21_Numbers,15,21,46,57);
	//glcd_set_font(Liberation_Sans27x36_Numbers,27,36,46,57);
	//glcd_set_font(Liberation_Sans17x17_Alpha,17,17,46,90);

	max_y = GLCD_LCD_HEIGHT - font_current.height - 2; // max y start position for draw_string

	while(1) {
		// move top to bottom
		for (y=0; y<max_y; y++) {
			DEMO_RETURN();
			glcd_clear_buffer();
			glcd_draw_string_xy(0,y,"123");
			glcd_write();
			delay_ms(80);
		}

		// move bottom to top
		for (y=(max_y); y>0; y--) {
			DEMO_RETURN();
			glcd_clear_buffer();
			glcd_draw_string_xy(0,y,"456");
			glcd_write();
			delay_ms(80);
		}
		
	}

}

void glcd_test_tiny_text(void)
{
	/* Write tiny text on display, all chars, scrolling up every second */
	
	char string[GLCD_LCD_WIDTH / 6 + 1];

	uint8_t c = 32;
	uint8_t len = GLCD_LCD_WIDTH / 6;
	
	GLCD_TEXT_INIT();
	
	while(1) {
		// write chars to string from 32 to 127 ASCII
		uint8_t i;
		for (i=0; i<len; i++) {
			string[i] = c;
			c++;
			if (c > 127) {
				c = 32;
			}
		}

		// write null terminator
		string[len] = '\0';

		GLCD_WRITE(string);
		
		DEMO_RETURN();
		
		delay_ms(1000);
	}

}

void glcd_test_hello_world(void)
{
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	glcd_tiny_draw_string(0,0,"Hello World!0");
	glcd_tiny_draw_string(0,1,"Hello World!1");
	glcd_tiny_draw_string(0,2,"Hello World!2");
	glcd_tiny_draw_string(0,3,"Hello World!3");
	glcd_tiny_draw_string(0,4,"Hello World!4");
	glcd_tiny_draw_string(0,5,"Hello World!5");
	glcd_tiny_draw_string(0,6,"Hello World!6");
	glcd_tiny_draw_string(0,7,"Hello World!7");
	glcd_write();
	while(1) {
		DEMO_RETURN();
	}
}

void glcd_test_rectangles(void)
{
	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	glcd_tiny_draw_string(0,0,"RECTANGLE DEMO");
	glcd_write();
	delay_ms(200);
	

	while(1) {
		
		glcd_clear();
		glcd_draw_rect(0,0,100,50,BLACK);
		glcd_tiny_draw_string(0,GLCD_NUMBER_OF_BANKS-1,"glcd_draw_rect");
		glcd_write();
		delay_ms(500);
		DEMO_RETURN();
		
		glcd_clear();
		glcd_tiny_draw_string(0,GLCD_NUMBER_OF_BANKS-1,"glcd_draw_rect_thick");
		glcd_write();

		glcd_draw_rect_thick(5,5,80,30,3,6,BLACK);
		glcd_write();
		delay_ms(500);
		DEMO_RETURN();
		
		glcd_draw_rect_thick(0,0,20,20,2,2,BLACK);
		glcd_write();
		delay_ms(500);
		DEMO_RETURN();
		
		glcd_draw_rect_thick(100,10,20,20,5,5,BLACK);
		glcd_write();
		delay_ms(500);
		DEMO_RETURN();

		glcd_clear();
		glcd_tiny_draw_string(0,GLCD_NUMBER_OF_BANKS-1,"glcd_draw_rect_shadow");
		glcd_draw_rect_shadow(0,0,45,30,BLACK);
		glcd_write();
		delay_ms(500);
		DEMO_RETURN();
		
		glcd_draw_rect_shadow(95,5,30,30,BLACK);
		glcd_write();
		delay_ms(500);
		DEMO_RETURN();
		
	}
	
}

void glcd_test_scrolling_graph(void)
{
	glcd_clear();
	glcd_write();
	while(1) {
		uint16_t n;
		for (n=0; n<=255; n += 20) {
			glcd_scrolling_bar_graph(0,0,50,50,n);
			glcd_scrolling_bar_graph(60,0,50,30,n);
			glcd_scrolling_bar_graph(60,35,60,20,n);
			DEMO_RETURN();
		}
		for (n=0; n<=255; n += 20) {
			glcd_scrolling_bar_graph(0,0,50,50,255-n);
			glcd_scrolling_bar_graph(60,0,50,30,n);
			glcd_scrolling_bar_graph(60,35,60,20,n);
			DEMO_RETURN();
		}
	}	
}

#if defined (GLCD_UNIT_TEST_BITMAP_ENABLE)
void glcd_test_bitmap_128x64(void)
{
	glcd_draw_bitmap(bmp_oslogo);
	glcd_write();	
	while (1) {
		DEMO_RETURN();
	}
}
#endif
