/*
 * TMC5160_driver.c
 *
 *  Created on: Feb 5, 2022
 *      Author: nkusanda
 */


#include "main.h"
#include "cmsis_os.h"
#include "TMC5160_driver.h"


void setup_motor(void){
	uint8_t spi_buf[5];

	// SETUP LEFT MOTOR - CS_0 (PK1)
	HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1); // start CS pin at 1

	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0x80;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x0C; // GCONF
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);


	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xEC;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0xC3; // CHOPCONF
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0x90;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x08;
	  spi_buf[3] = 0x0F;
	  spi_buf[4] = 0x0A; // IHOLD IRUN
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0x91;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x0A; // TPOWERDOWN
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0x93;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x01;
	  spi_buf[4] = 0xF4; // TPWMTHRS
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xA6;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x03;
	  spi_buf[4] = 0xE8; // AMAX = 1000
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0);// go low to start transmission
	  spi_buf[0] = 0xA7;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x86;
	  spi_buf[4] = 0xA0; // VMAX = 100 000
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);




	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // go low to start transmission
	  spi_buf[0] = 0xB4;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x03; // activate stop if REFL/R enabled
	  	  	  	  	  	  // if rotating right, REFR=1 stops
	  	  	  	  	  	  // if rotating left, REFL=1 stops
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);



	// SETUP RIGHT MOTOR - CS_1 (PA13)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1); // start CS pin at 1
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // go low to start transmission
	  spi_buf[0] = 0x80;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x0C; // GCONF
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);



	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // go low to start transmission
	  spi_buf[0] = 0xEC;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0xC3; // CHOPCONF
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);



	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // go low to start transmission
	  spi_buf[0] = 0x90;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x08;
	  spi_buf[3] = 0x0F;
	  spi_buf[4] = 0x0A; // IHOLD IRUN
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);



	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // go low to start transmission
	  spi_buf[0] = 0x91;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x0A; // TPOWERDOWN
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);



	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // go low to start transmission
	  spi_buf[0] = 0x93;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x01;
	  spi_buf[4] = 0xF4; // TPWMTHRS
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);


	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // go low to start transmission
	  spi_buf[0] = 0xA6;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x03;
	  spi_buf[4] = 0xE8; // AMAX = 1000
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);



	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0);// go low to start transmission
	  spi_buf[0] = 0xA7;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x01;
	  spi_buf[3] = 0x86;
	  spi_buf[4] = 0xA0; // VMAX = 100 000
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);



	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // go low to start transmission
	  spi_buf[0] = 0xB4;
	  spi_buf[1] = 0x00;
	  spi_buf[2] = 0x00;
	  spi_buf[3] = 0x00;
	  spi_buf[4] = 0x03; // activate stop if REFL/R enabled
	  	  	  	  	  	  // if rotating right, REFR=1 stops
	  	  	  	  	  	  // if rotating left, REFL=1 stops
	  HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);
}

void rotate(int clockwise, int left_or_right){
	// anti-clockwise = 0, left = 0,
	uint8_t spi_buf[5];

	// left arm, close
	if (clockwise == 1 && left_or_right == 0){
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // CS_0 for left (PK1) go low to start transmission
		spi_buf[0] = 0xA0;
		spi_buf[1] = 0x00;
		spi_buf[2] = 0x00;
		spi_buf[3] = 0x00;
		spi_buf[4] = 0x01; // RAMPMODE = 1 - clockwise
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);
	}

	// right arm, open
	if (clockwise == 1 && left_or_right == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // CS_1 for right (PA13) go low to start transmission
		spi_buf[0] = 0xA0;
		spi_buf[1] = 0x00;
		spi_buf[2] = 0x00;
		spi_buf[3] = 0x00;
		spi_buf[4] = 0x01; // RAMPMODE = 1 - clockwise
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);
	}

	// left arm, open
	if (clockwise == 0 && left_or_right == 0){
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 0); // CS_0 for left (PK1) go low to start transmission
		spi_buf[0] = 0xA0;
		spi_buf[1] = 0x00;
		spi_buf[2] = 0x00;
		spi_buf[3] = 0x00;
		spi_buf[4] = 0x02; // RAMPMODE = 2 - anti-clockwise
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, 1);
	}

	// right arm, close
	if (clockwise == 0 && left_or_right == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 0); // CS_1 for right (PA13) go low to start transmission
		spi_buf[0] = 0xA0;
		spi_buf[1] = 0x00;
		spi_buf[2] = 0x00;
		spi_buf[3] = 0x00;
		spi_buf[4] = 0x02; // RAMPMODE = 2 - anti-clockwise
		HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *)spi_buf, 5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, 1);
	}


}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

/**
  * @brief Rx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
}
