/*
 * bms.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */

#include "bms.h"
#include "main.h"

bool bms_init(
		Bms* this,
		SPI_HandleTypeDef* _spi_handle,
		GPIO_TypeDef* _spi_cs_port,
		uint16_t _spi_cs_pin
		)
{
	return 0;
}

static bool _update_all_bms_modules(Bms* this) {
	return 0;
}
