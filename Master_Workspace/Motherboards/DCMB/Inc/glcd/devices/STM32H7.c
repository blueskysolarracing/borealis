/**
	\file STM32F4.h
	\author Andy Gock (modified for STM32F4xx by Moreto)
	\brief Functions specific to STM32 F4 ARM Cortex-M4 devices.
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

#if defined(GLCD_DEVICE_STM32H7XX)

/* Includes from CMSIS and Peripheral Library */
#include "devices/STM32H7.h"
//#include "stm32h7xx_spi.h"
#include "stm32h7xx_hal_spi.h"
#include "main.h"
/* Includes from GLCD */
#include "glcd.h"

/* Includes for RTOS */
#ifdef GLCD_USE_RTOS
  //#include "BRTOS.h"
#include "cmsis_os.h"
#endif

void delay_ms(uint32_t ms);

//#define BACKLIGHT_INVERT	// Uncomment if LED backlight turn on with low value

void glcd_init(void)
{

	/* Initialization for PCD8544 controller */

	/* Declare GPIO and SPI init structures */
	//NOT NEEDED BECAUSE WE HAVE EVERYTHING ALREADY
//	GPIO_InitTypeDef GPIO_InitStructure;
//	SPI_InitTypeDef  SPI_InitStructure;
//
//	/* Configuring CS, DC (A0) and RST pin */
//	/* Peripheral clock init. */
//	RCC_AHB1PeriphClockCmd(CONTROLLER_SPI_DC_RCC | CONTROLLER_SPI_SS_RCC |
//			CONTROLLER_SPI_RST_RCC, ENABLE);
//	/* CS pin */
//	GPIO_InitStructure.GPIO_Pin   = CONTROLLER_SPI_SS_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_Init(CONTROLLER_SPI_SS_PORT, &GPIO_InitStructure);
//
//	/* DC (A0) pin */
//	GPIO_InitStructure.GPIO_Pin = CONTROLLER_SPI_DC_PIN;
//	GPIO_Init(CONTROLLER_SPI_DC_PORT, &GPIO_InitStructure);
//
//	/* RST pin */
//	GPIO_InitStructure.GPIO_Pin = CONTROLLER_SPI_RST_PIN;
//	GPIO_Init(CONTROLLER_SPI_RST_PORT, &GPIO_InitStructure);

	/* Make sure chip is de-selected by default */
	GLCD_DESELECT_P1();
	GLCD_DESELECT_P2();

	/*
	 * Configuring SPI:
	 */
	/* Enable the SPI clock */
	//ALREADY SET UP BY DEFAULT
//	SPIx_CLK_INIT(SPIx_CLK, ENABLE);
//	/* Enable SPI GPIO clocks */
//	RCC_AHB1PeriphClockCmd(SPIx_SCK_GPIO_CLK | SPIx_MOSI_GPIO_CLK, ENABLE);
//	/* Connect SPI pins to the corresponding alternate function */
//	GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
//	GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);
////	GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);
//
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
//
//	/* SPI SCK pin configuration */
//	GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
//	GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);
//	/* SPI  MOSI pin configuration */
//	GPIO_InitStructure.GPIO_Pin =  SPIx_MOSI_PIN;
//	GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
//#if 0
//	GPIO_InitStructure.GPIO_Pin =  SPIx_MISO_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
//#endif
//	/* SPI configuration */
//	SPI_I2S_DeInit(SPIx);
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	/* Clock set to 8MHz */
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_PRESCALLER;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//	SPI_Init(SPIx, &SPI_InitStructure);
//	/* Enable the SPI peripheral */
//	SPI_Cmd(SPIx, ENABLE);

	glcd_select_screen((uint8_t *)&glcd_buffer,&glcd_bbox);

	//ALREADY CONFIGURED
//	RCC_AHB1PeriphClockCmd(LCD_LED_GPIO_RCC, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = LCD_LED_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//	GPIO_Init(LCD_LED_PORT, &GPIO_InitStructure);


#if defined(GLCD_CONTROLLER_PCD8544)
	/* Initialisation sequence of controller */

	glcd_PCD8544_init();
	glcd_clear();

#elif defined(GLCD_CONTROLLER_ST7565R)
	glcd_reset();
	glcd_enable_backlight(ENABLE);
	pToggle = 0;
	glcd_ST7565R_init();
	pToggle = 1;
	glcd_ST7565R_init();

#else
	#error "Controller not supported by STM32F0xx"
#endif

}

/* Change backlight led state ENABLE or DISABLE*/
void glcd_enable_backlight(FunctionalState state)
{

#ifdef USE_TIMER_PWM
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_Base_InitTypeDef  TIM_TimeBaseStructure;
	TIM_OC_InitTypeDef  TIM_OCInitStructure;

	uint16_t PrescalerValue = 0;
	/* Init LED backlight pin*/
	GPIO_InitStructure.Pin = DISP_LED_CTRL_Pin;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;

	if (state == ENABLE){

		/* Compute the prescaler value
		 * This formula yelds the prescaller for a frequency of 100Hz with
		 * 255 steps (period) */
		PrescalerValue = (uint16_t) (((SystemCoreClock /LCD_TIM_Sys_Prescaler) / 255)/100) - 1;

		/* Init LED pin as Alternated Function: */
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		HAL_GPIO_Init(DISP_LED_CTRL_GPIO_Port, &GPIO_InitStructure);

		/* Connect TIM2 pins to AF */
		HAL_GPIO_PinAFConfig(DISP_LED_CTRL_GPIO_Port, DISP_LED_CTRL_Pin, LCD_AF);

		/* TIM clock enable */
		LCD_TIM_PeriphClockCmd(LCD_TIM_RCC, ENABLE);

		/* Time base configuration: 100Hz period with 255 steps */
		TIM_TimeBaseStructure.Period = 254;
		TIM_TimeBaseStructure.Prescaler = PrescalerValue;
		TIM_TimeBaseStructure.ClockDivision = 0;
		TIM_TimeBaseStructure.CounterMode = TIM_COUNTERMODE_UP;
		HAL_TIM_Base_Init(LCD_TIM, &TIM_TimeBaseStructure);

		/* PWM1 Mode configuration: Channel3 */
		TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
		TIM_OCInitStructure.Pulse = 128; // Initial value (half)
#ifdef BACKLIGHT_INVERT_OUT
		TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
#else
		TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
#endif
		LCD_TIM_OCInit(LCD_TIM, &TIM_OCInitStructure);
		LCD_TIM_OCPreload(LCD_TIM, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(LCD_TIM, ENABLE);

		/* TIM3 enable counter */
		TIM_Cmd(LCD_TIM, ENABLE);

	}
	/* If DISABLE: Disable timer and turn off led */
	else{
		// DeInit LED pin as Alternated Function:
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_Init(DISP_LED_CTRL_GPIO_Port, &GPIO_InitStructure);
		/* TIM clock disable */
		TIM_Cmd(LCD_TIM, DISABLE);
		LCD_TIM_PeriphClockCmd(LCD_TIM_RCC, DISABLE);
#ifdef BACKLIGHT_INVERT_OUT
		GPIO_SetBits(DISP_LED_CTRL_GPIO_Port,DISP_LED_CTRL_Pin);
#else
		// Turn off LED backlight
		GPIO_ResetBits(DISP_LED_CTRL_GPIO_Port,DISP_LED_CTRL_Pin);
	}
#endif
#else // if not defined USE_TIMER_PWM
	if (state == ENABLE){
	#ifdef BACKLIGHT_INVERT_OUT
		GPIO_ResetBits(DISP_LED_CTRL_GPIO_Port,DISP_LED_CTRL_Pin);
	#else
		//GPIO_SetBits(DISP_LED_CTRL_GPIO_Port,DISP_LED_CTRL_Pin); TODO
	#endif
	}else{
	#ifdef BACKLIGHT_INVERT_OUT
		GPIO_SetBits(DISP_LED_CTRL_GPIO_Port,DISP_LED_CTRL_Pin);
	#else
		//GPIO_ResetBits(DISP_LED_CTRL_GPIO_Port,DISP_LED_CTRL_Pin); TODO
	#endif
	}
#endif
}

#ifdef USE_TIMER_PWM
/* Change PWM duty cycle (brightness).
 * values between 0 and 255 */
void glcd_change_backlight(uint8_t value){

	LCD_TIM_SetCompare(LCD_TIM,value);
}
#endif

void glcd_spi_write(uint8_t c)
{
	//uint8_t temp;
	switch (pToggle){
		case 1:
			GLCD_SELECT_P2();
			break;
		default:
			GLCD_SELECT_P1();
			break;
	}

	/*!< Loop while DR register in not emplty */
	//OLD while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);

	//OLD SPI_I2S_SendData(SPIx, (uint16_t) c);
	HAL_SPI_Transmit(&hspi2, &c, 1, MAX_SPI_TRANSMIT_TIMEOUT);
//	HAL_SPI_Transmit_IT(&hspi2, &c, 1);
	

	/* Wait until entire byte has been read (which we discard anyway) */
	//OLD while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) != RESET);

	//temp = SPI_I2S_ReceiveData(SPIx);

	switch (pToggle){
		case 1:
			GLCD_DESELECT_P2();
			break;
		default:
			GLCD_DESELECT_P1();
			break;
	}
}

void glcd_reset(void)
{
	/* Toggle RST low to reset. Minimum pulse 100ns on datasheet. */
	GLCD_SELECT_P1();
	GLCD_SELECT_P2();
	GLCD_RESET_LOW_P1();
	GLCD_RESET_LOW_P2();

	osDelay(1000);
	//DelayTask(GLCD_RESET_TIME);
	GLCD_RESET_HIGH_P1();
	GLCD_RESET_HIGH_P2();
	GLCD_DESELECT_P1();
	GLCD_DESELECT_P2();
}

void delay_ms(uint32_t ms){

#ifndef GLCD_USE_RTOS
	uint32_t count = 0;
	uint32_t ms_counter = 0;
	do{
		count = 0;
		const uint32_t utime = 100000;
		do{
			count++;
		}
		while (count < utime);
		ms_counter++;
	}while(ms_counter < ms);
#else
	GLCD_RTOS_DELAY_FCN // Call the delay function defined in the header file.
#endif
}


#endif
