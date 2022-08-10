/*
 * TMC5160_driver.c
 *
 *  Created on: Feb 5, 2022
 *      Author: nkusanda
 */


#include "main.h"
#include "TMC5160_driver.h"
int state;
int flag;

void setup_motor(SPI_HandleTypeDef *hspi5){
	uint8_t spi_buf[5] = {0x80, 0x00, 0x00, 0x00, 0x0C};
	int state = 0;
	int flag = 0; // 0 - clear flag
	// SETUP LEFT MOTOR - CS_0 (PK1)
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1); // start CS pin at 1
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
//	  spi_buf[0] = 0x80;
//	  spi_buf[1] = 0x00;
//	  spi_buf[2] = 0x00;
//	  spi_buf[3] = 0x00;
//	  spi_buf[4] = 0x0C; // GCONF
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
//	  spi_buf[0] = 0x80;
//	  spi_buf[1] = 0x00;
//	  spi_buf[2] = 0x00;
//	  spi_buf[3] = 0x00;
//	  spi_buf[4] = 0x0C; // GCONF
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xEC;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0xC3; // CHOPCONF
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0x90;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x08;
	  spi_buf[3] = 0x0F;
	  spi_buf[4] = 0x0A; // IHOLD IRUN
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0x91;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x0A; // TPOWERDOWN
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0x93;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x01;
	  spi_buf[4] = 0xF4; // TPWMTHRS
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xA6;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x03;
	  spi_buf[4] = 0xE8; // AMAX = 1000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xA8;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x4E;
	  spi_buf[4] = 0x20; // DMAX = 2000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xAA;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x07;
	  spi_buf[4] = 0xD0; // D1 = 2000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xA7;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x86;
	  spi_buf[4] = 0xA0; // VMAX = 100 000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xB4;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x03; // activate stop if REFL/R enabled
	  	  	  	  	  	  // if rotating right, REFR=1 stops
	  	  	  	  	  	  // if rotating left, REFL=1 stops
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);




	// SETUP RIGHT MOTOR - CS_1 (PK4)
	HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1); // start CS pin at 1
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0x80;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x0C; // GCONF
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0xEC;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0xC3; // CHOPCONF
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0x90;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x08;
	  spi_buf[3] = 0x0F;
	  spi_buf[4] = 0x0A; // IHOLD IRUN
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0x91;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x0A; // TPOWERDOWN
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0x93;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x01;
	  spi_buf[4] = 0xF4; // TPWMTHRS
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0xA6;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x03;
	  spi_buf[4] = 0xE8; // AMAX = 1000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0xA6;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x13;
	  spi_buf[4] = 0x88; // DMAX = 2000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);
	  
	  
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0xAA;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x07;
	  spi_buf[4] = 0xD0; // D1 = 2000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);


	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0xA7;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x86;
	  spi_buf[4] = 0xA0; // VMAX = 100 000
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // go low to start transmission
	  spi_buf[0] = 0xB4;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x03; // activate stop if REFL/R enabled
	  	  	  	  	  	  // if rotating right, REFR=1 stops
	  	  	  	  	  	  // if rotating left, REFL=1 stops
	  HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);

}

void rotate(int clockwise, int left_or_right, SPI_HandleTypeDef *hspi5){
	// anti-clockwise = 0, left = 0,
	uint8_t spi_buf[5] = {0xA0, 0x00, 0x00, 0x00, 0x01}; // RAMPMODE = 1 - clockwise

	// left arm, close
	if (clockwise == 1 && left_or_right == 0){
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // CS_0 for left (PK1) go low to start transmission
//		spi_buf[0] = 0xA0;
//		spi_buf[1] = 0x00;
//		spi_buf[2] = 0x00;
//		spi_buf[3] = 0x00;
//		spi_buf[4] = 0x01; // RAMPMODE = 1 - clockwise
		HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);
	}

	// right arm, open
	else if (clockwise == 1 && left_or_right == 1){
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // CS_1 for right (PK4) go low to start transmission
//		spi_buf[0] = 0xA0;
//		spi_buf[1] = 0x00;
//		spi_buf[2] = 0x00;
//		spi_buf[3] = 0x00;
//		spi_buf[4] = 0x01; // RAMPMODE = 1 - clockwise
		HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);
	}

	// left arm, open
	else if (clockwise == 0 && left_or_right == 0){
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // CS_0 for left (PK1) go low to start transmission
		spi_buf[0] = 0xA0;
		spi_buf[1] = 0x00;
		spi_buf[2] = 0x00;
		spi_buf[3] = 0x00;
		spi_buf[4] = 0x02; // RAMPMODE = 2 - anti-clockwise
		HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);
	}

	// right arm, close
	else if (clockwise == 0 && left_or_right == 1){
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 0); // CS_1 for right (PK4) go low to start transmission
		spi_buf[0] = 0xA0;
		spi_buf[1] = 0x00;
		spi_buf[2] = 0x00;
		spi_buf[3] = 0x00;
		spi_buf[4] = 0x02; // RAMPMODE = 2 - anti-clockwise
		HAL_SPI_Transmit(hspi5, (uint8_t *)spi_buf, 5, 100);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, 1);
	}
}


/**
  * @brief Rx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */

