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
	bool (*get_temperature)(struct Bms* this, float* temperatures, uint8_t bms_module_id, get_mode_t get_mode);
	bool (*get_voltage)(struct Bms* this, float* voltages, uint8_t bms_module_id, get_mode_t get_mode);
	bool (*get_state_of_charge)(struct Bms* this, float* state_of_charges, uint8_t bms_module_id);
	void (*set_current)(struct Bms* this, float current);

	// Note: bms_module_id ranges from 0 - 5, each representing a physical battery module

	void (*run)(struct Bms* this); // creates and starts thread which updates bms_modules in the background
	void (*stop)(struct Bms* this); // deletes the run thread. Call this before the bms object is deleted

	/* private */
	BmsModule _bms_modules[BMS_NUM_BMS_MODULES];
	TaskHandle_t _run_thread_handle;


}Bms;

void bms_init(
		Bms* this,
		SPI_HandleTypeDef* spi_handle,
		GPIO_TypeDef* spi_cs_ports[],
		uint16_t spi_cs_pins[]
		);
// Note: spi related arguments will be passed into each BmsModule



#endif /* INC_BMS_H_ */
