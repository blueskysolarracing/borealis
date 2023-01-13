/*
 * bms.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

#include "bms_module.h"
#include "stdbool.h"
#include "main.h"

#define BMS_NUM_BMS_MODULES 6

typedef struct Bms {

	/* public */
	void (*get_temperature_array)(struct Bms* this, float* temperature_array, int bms_module_id);
	void (*get_voltage_array)(struct Bms* this, float* voltage_array, int bms_module_id);
	void (*get_soc_array)(struct Bms* this, float* soc_array, int bms_module_id);

	// Note: bms_module_id ranges from 0 - 5

	bool (*run)(struct Bms* this); // Starts thread which updates bms_modules in the background

	/* private */
	BmsModule _bms_modules[BMS_NUM_BMS_MODULES];
	TaskHandle_t _run_thread_handle;


}Bms;

bool bms_init(
		Bms* this,
		SPI_HandleTypeDef* _spi_handle,
		GPIO_TypeDef* _spi_cs_port,
		uint16_t _spi_cs_pin
		);
// Note: spi related arguments will be passed into each BmsModule



#endif /* INC_BMS_H_ */
