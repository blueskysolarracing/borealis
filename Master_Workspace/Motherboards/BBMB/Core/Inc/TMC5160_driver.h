/*
 * TMC5160_driver.h
 *
 *  Created on: Feb 5, 2022
 *      Author: nkusanda
 */

#include "main.h"
#include "cmsis_os.h"

void setup_motor(void);
void rotate(int clockwise, int left_or_right);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
