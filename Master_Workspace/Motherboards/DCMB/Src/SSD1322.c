/*
 * SSD1322.c
 *
 *  Created on: Oct 5, 2019
 *      Author: jamesliu
 */

#include "SSD1322.h"

static void setCS(uint8_t x);
static void setDC(uint8_t x);
static void spiSendByte(uint8_t x);
static void spiSendBuf(uint8_t* buf, size_t len);
SemaphoreHandle_t txCpltSemHack;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
static SPI_HandleTypeDef* hspi = &hspi2; //SPI2
static SemaphoreHandle_t txCpltSem;
GPIO_TypeDef* dcPort = GPIOI; //MISO 156 I2
uint32_t dcPin = GPIO_PIN_2;
GPIO_TypeDef* csPort = GPIOI; //CS0 154 I0
uint32_t csPin = GPIO_PIN_0;

extern const uint8_t kaboom[8192];

void SSD_init(){
	txCpltSem = xSemaphoreCreateBinary();
	xSemaphoreGive(txCpltSem);

	setCS(0);

	setDC(0);
	spiSendByte(0xfd); // unlock commands
	setDC(1);
	spiSendByte(0x12);

	setDC(0);
	spiSendByte(0xae); // sleep mode on (disp off)

	setDC(0);
	spiSendByte(0xb3); // oscillator stuff str8 from datasheet
	setDC(1);
	spiSendByte(0x91); // default good boy frequency
//	spiSendByte(0xf1); // MAXXXED OUT LET'S GO (but still dividing by 2 for DFF stability)

	setDC(0);
	spiSendByte(0xca); // mux ratio 1/64 (64 rows)
	setDC(1);
	spiSendByte(0x3f);

	setDC(0);
	spiSendByte(0xa2); // display offset 0
	setDC(1);
	spiSendByte(0x00);

	setDC(0);
	spiSendByte(0xa1); // display start line 0
	setDC(1);
	spiSendByte(0x00);

	setDC(0);
	spiSendByte(0xa0);
	setDC(1);
	spiSendByte(0x14); // row order reversed; nybble order remapped ({0x01, 0x23})
	setDC(1);
	spiSendByte(0x11); // dual COM line enabled

	setDC(0);
	spiSendByte(0xab); // enable internal VDD regulator
	setDC(1);
	spiSendByte(0x01);

	setDC(0);
	spiSendByte(0xb4);
	setDC(1);
	spiSendByte(0xa0); // enable external VSL
	setDC(1);
	spiSendByte(0xfd); // enhanced low GS display quality enabled

	setDC(0);
	spiSendByte(0xc1); // I_seg current = maximum
	setDC(1);
	spiSendByte(0xff);

	setDC(0);
	spiSendByte(0xc7); // no current reduction 16/16
	setDC(1);
	spiSendByte(0x0f);

	setDC(0);
	spiSendByte(0xb9); // use default Linear Gray Scale table

	setDC(0);
	spiSendByte(0xb1); // drive phase stuff str8 from datasheet
	setDC(1);
	spiSendByte(0xe2);

	setDC(0);
	spiSendByte(0xbb); // precharge voltage = max
	setDC(1);
	spiSendByte(0x1f);

	setDC(0);
	spiSendByte(0xb6); // precharge period = middle (default)
	setDC(1);
	spiSendByte(0x08);

	setDC(0);
	spiSendByte(0xbe); // COM deselect voltage = max (0.86 Vcc)
	setDC(1);
	spiSendByte(0x07);

	setDC(0);
	spiSendByte(0xa7); // normal display mode (not blank or inverted)

	setDC(0);
	spiSendByte(0xaf); // sleep mode off (disp on)

	setCS(1);

	lv_area_t area;
	area.x1 = 0;
	area.x2 = 255;
	area.y1 = 0;
	area.y2 = 63;
	SSD_writeRegion(&area, kaboom);

	osDelay(2000);
}

void SSD_init_hack(){
	txCpltSemHack = xSemaphoreCreateBinary();
	xSemaphoreGive(txCpltSemHack);

//	osDelay(2); // let arduino start up

//	lv_area_t area;
//	area.x1 = 0;
//	area.x2 = 255;
//	area.y1 = 0;
//	area.y2 = 63;
//	SSD_writeRegion_hack(&area, kaboom);

//	osDelay(2000);
}

void SSD_writeRegion(lv_area_t * area, uint8_t* buf){
	setCS(0);
	setDC(0);
	spiSendByte(0x75); // set row address
	setDC(1);
	spiSendByte(0+area->y1); // min 0
	setDC(1);
	spiSendByte(0+area->y2); // max 63
	setDC(0);
	spiSendByte(0x15); // set col address
	setDC(1);
	spiSendByte(28+area->x1/4); // min 28
	setDC(1);
	spiSendByte(28+(area->x2)/4); // max 91
	setDC(0);
	spiSendByte(0x5c); // write RAM
	setDC(1);
	size_t dx = area->x2-area->x1+1;
	size_t dy = area->y2-area->y1+1;
	size_t len = dx*dy/2;
//	for(int i=0; i<len; i++){
//		spiSendByte(*buf++);
//	}
	spiSendBuf(buf, len);
	setCS(1);
}

void SSD_writeRegion_hack(lv_area_t * area, uint8_t* buf){
	static uint8_t* dmabuf = NULL;
	vPortFree(dmabuf);
	size_t dx = area->x2-area->x1+1;
	size_t dy = area->y2-area->y1+1;
	size_t len = dx*dy/2;
	uint8_t buf2[5] = {0xa5, 0+area->y1, 0+area->y2, 28+area->x1/4, 28+(area->x2)/4};
	xSemaphoreTake(txCpltSemHack, portMAX_DELAY);
	HAL_UART_Transmit(&huart2, buf2, 5, 10);
	dmabuf = pvPortMalloc(len);
	memcpy(dmabuf, buf, len);
	HAL_UART_Transmit_DMA(&huart2, dmabuf, len);
}

void SSD_test(){
	const uint8_t A[60] = {0xff,0xf4,0xf,0xff,0xff,0xd0,0xa,0xff,0xff,0xa2,0x77,0xff,0xff,0x57,0xa2,0xff,0xff,0xb,0xd0,0xcf,0xfb,0xf,0xf2,0x9f,0xf7,0x0,0x0,0x2f,0xf2,0x9f,0xfc,0xd,0xc0,0xdf,0xff,0x2a,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

	const uint8_t B[60] = {0xf8,0x0,0x5,0xdf,0xf8,0x6f,0xf7,0x5f,0xf8,0x6f,0xfa,0xf,0xf8,0x6f,0xf5,0x6f,0xf8,0x0,0x0,0xdf,0xf8,0x6f,0xfa,0x2f,0xf8,0x6f,0xfe,0xc,0xf8,0x6f,0xfa,0xe,0xf8,0x0,0x2,0xaf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

	const uint8_t C[60] = {0xff,0xc5,0x0,0x8f,0xfe,0x28,0xff,0x7f,0xf9,0x2f,0xff,0xff,0xf6,0x6f,0xff,0xff,0xf5,0x8f,0xff,0xff,0xf6,0x6f,0xff,0xff,0xf9,0x2f,0xff,0xff,0xfe,0x8,0xff,0xaf,0xff,0xc5,0x0,0x8f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

	const uint8_t D[60] = {0xf4,0x0,0x2a,0xff,0xf4,0x8f,0xc0,0xaf,0xf4,0x8f,0xf8,0x5f,0xf4,0x8f,0xfa,0xf,0xf4,0x8f,0xfb,0xe,0xf4,0x8f,0xfa,0xf,0xf4,0x8f,0xf8,0x5f,0xf4,0x8f,0xc0,0xaf,0xf4,0x0,0x2a,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

	const uint8_t E[60] = {0xfa,0x0,0x0,0xf,0xfa,0xf,0xff,0xff,0xfa,0xf,0xff,0xff,0xfa,0xf,0xff,0xff,0xfa,0x0,0x0,0x6f,0xfa,0xf,0xff,0xff,0xfa,0xf,0xff,0xff,0xfa,0xf,0xff,0xff,0xfa,0x0,0x0,0xe,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

	SSD_clearDisplay();

	setCS(0);
	setDC(0);
	spiSendByte(0x75); // set row address
	setDC(1);
	spiSendByte(0);
	setDC(1);
	spiSendByte(64);
	setDC(0);
	spiSendByte(0x15); // set col address
	setDC(1);
	spiSendByte(28);
	setDC(1);
	spiSendByte(91);
	setDC(0);
	spiSendByte(0x5c);
	setDC(1);
	uint8_t* buf1 = pvPortMalloc(129*2);
	uint8_t* buf1p = buf1;
	for(int i=0; i<129; i++){
		*buf1p++ = 0xf4;
		*buf1p++ = 0x84;
	}
	spiSendBuf(buf1, 129*2);
	vPortFree(buf1);
	setCS(1);

	osDelay(1);

//	setCS(0);
//	setDC(0);
//	spiSendByte(0x75); // set row address
//	setDC(1);
//	spiSendByte(4);
//	setDC(1);
//	spiSendByte(16);
//	setDC(0);
//	spiSendByte(0x15); // set col address
//	setDC(1);
//	spiSendByte(28);
//	setDC(1);
//	spiSendByte(29);
//	setDC(0);
//	spiSendByte(0x5c);
//	setDC(1);
//	for(int i=0; i<52; i++){
//		spiSendByte(~A[i]);
//	}
//
//	spiSendBuf(A, 52);
	lv_area_t area;
	area.x1 = 0;
	area.x2 = 7;
	area.y1 = 4;
	area.y2 = 16;
	SSD_writeRegion(&area, A);
	setCS(0);
	setDC(0);
	spiSendByte(0x15); // set col address
	setDC(1);
	spiSendByte(30);
	setDC(1);
	spiSendByte(31);
	setDC(0);
	spiSendByte(0x5c);
	setDC(1);
//	for(int i=0; i<52; i++){
//		spiSendByte(~B[i]);
//	}
	spiSendBuf(B, 52);
	setDC(0);
	spiSendByte(0x15); // set col address
	setDC(1);
	spiSendByte(32);
	setDC(1);
	spiSendByte(33);
	setDC(0);
	spiSendByte(0x5c);
	setDC(1);
//	for(int i=0; i<52; i++){
//		spiSendByte(~C[i]);
//	}
	spiSendBuf(C, 52);
	setDC(0);
	spiSendByte(0x15); // set col address
	setDC(1);
	spiSendByte(34);
	setDC(1);
	spiSendByte(35);
	setDC(0);
	spiSendByte(0x5c);
	setDC(1);
//	for(int i=0; i<52; i++){
//		spiSendByte(~D[i]);
//	}
	spiSendBuf(D, 52);
	setDC(0);
	spiSendByte(0x15); // set col address
	setDC(1);
	spiSendByte(36);
	setDC(1);
	spiSendByte(37);
	setDC(0);
	spiSendByte(0x5c);
	setDC(1);
//	for(int i=0; i<52; i++){
//		spiSendByte(~E[i]);
//	}
	spiSendBuf(E, 52);
	setCS(1);

	osDelay(1);

	setCS(0);
	setDC(0);
	spiSendByte(0x75); // set row address
	setDC(1);
	spiSendByte(64);
	setDC(1);
	spiSendByte(127);
	setDC(0);
	spiSendByte(0x15); // set col address
	setDC(1);
	spiSendByte(28);
	setDC(1);
	spiSendByte(91);
	setDC(0);
	spiSendByte(0x5c);
	setDC(1);
	uint8_t* buf2 = pvPortMalloc(129*2);
	uint8_t* buf2p = buf2;
	for(int i=0; i<129; i++){
		*buf2p++ = 0xff;
		*buf2p++ = 0x44;
	}
	spiSendBuf(buf2, 129*2);
	vPortFree(buf2);
	setCS(1);

	for(;;){
		osDelay(1000);
		setCS(0);
		setDC(0);
		spiSendByte(0xa1); // set display start line
		setDC(1);
		spiSendByte(64);
		setCS(1);
		osDelay(1000);
		setCS(0);
		setDC(0);
		spiSendByte(0xa1); // set display start line
		setDC(1);
		spiSendByte(0);
		setCS(1);
	}
}

void my_rounder_cb(lv_disp_drv_t* disp_drv, lv_area_t* area){
  /* Update the areas as needed. Can be only larger.
   * For example to always have lines 8 px height:*/
   area->x1 = area->x1 & 0xfffffffc;
   area->x2 = area->x2 | 0x3;
}

void my_disp_flush(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p){
	uint32_t dx = area->x2-area->x1+1;
	uint32_t dy = area->y2-area->y1+1;
	uint32_t len = dx*dy;
	uint8_t* buf = pvPortMalloc(len/2);
	uint8_t* bufp = buf;
#if LV_COLOR_DEPTH == 1
	for(size_t i = 0; i < dy; i++){
		for(size_t j = 0; j < dx; j+=4){
			*bufp = color_p++->full&0xf0;
			*bufp++ |= (color_p++->full&0xf0)>>4;
			*bufp = color_p++->full&0xf0;
			*bufp++ |= (color_p++->full&0xf0)>>4;
		}
	}
#else if LV_COLOR_DEPTH == 16
	for(size_t i = 0; i < dy; i++){
		for(size_t j = 0; j < dx; j+=4){
			*bufp = color_p++->ch.green>>2<<4;
			*bufp++ |= color_p++->ch.green>>2;
			*bufp = color_p++->ch.green>>2<<4;
			*bufp++ |= color_p++->ch.green>>2;
		}
	}
#endif
	SSD_writeRegion(area, buf);
	vPortFree(buf);
	lv_disp_flush_ready(disp_drv);         /* Indicate you are ready with the flushing*/
}

void my_disp_flush_hack(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p){
	uint32_t dx = area->x2-area->x1+1;
	uint32_t dy = area->y2-area->y1+1;
	uint32_t len = dx*dy;
	uint8_t* buf = pvPortMalloc(len/2);
	uint8_t* bufp = buf;
#if LV_COLOR_DEPTH == 1
	for(size_t i = 0; i < dy; i++){
		for(size_t j = 0; j < dx; j+=4){
			*bufp = color_p++->full&0xf0;
			*bufp++ |= (color_p++->full&0xf0)>>4;
			*bufp = color_p++->full&0xf0;
			*bufp++ |= (color_p++->full&0xf0)>>4;
		}
	}
#else if LV_COLOR_DEPTH == 16
	for(size_t i = 0; i < dy; i++){
		for(size_t j = 0; j < dx; j+=4){
			*bufp = color_p++->ch.green>>2<<4;
			*bufp++ |= color_p++->ch.green>>2;
			*bufp = color_p++->ch.green>>2<<4;
			*bufp++ |= color_p++->ch.green>>2;
		}
	}
#endif
	SSD_writeRegion_hack(area, buf);
	vPortFree(buf);
	lv_disp_flush_ready(disp_drv);         /* Indicate you are ready with the flushing*/
}

void SSD_clearDisplay(){
	setCS(0);
	setDC(0);
	spiSendByte(0x75);
	setDC(1);
	spiSendByte(0);
	setDC(1);
	spiSendByte(127);
	setDC(0);
	spiSendByte(0x15);
	setDC(1);
	spiSendByte(28);
	setDC(1);
	spiSendByte(91);
	setDC(0);
	spiSendByte(0x5c);
	setDC(1);
	uint8_t* buf = pvPortMalloc(16384); //128 rows * 64 col addr * 2 bytes per addr
	uint8_t* bufp = buf;
	for(int i=0; i<16384; i++){
		*bufp++ = 0;
	}
	setDC(1);
	spiSendBuf(buf, 16384);
	vPortFree(buf);
	setCS(1);
}

static void setCS(uint8_t x){
	HAL_GPIO_WritePin(csPort, csPin, x);
}

static void setDC(uint8_t x){
	HAL_GPIO_WritePin(dcPort, dcPin, x);
}

static void spiSendByte(uint8_t x){
	HAL_SPI_Transmit(hspi, &x, 1, 0);
}

static void spiSendBuf(uint8_t* buf, size_t len){
	xSemaphoreTake(txCpltSem, portMAX_DELAY);
	HAL_SPI_Transmit_DMA(hspi, buf, len);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == &hspi2){
		xSemaphoreGiveFromISR(txCpltSem, NULL);
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart2){
//		xSemaphoreGiveFromISR(txCpltSemHack, NULL);
//	}
//}
