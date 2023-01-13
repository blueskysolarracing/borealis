/*
 * bms_module.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */
#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"

#ifndef INC_BMS_MODULE_H_
#define INC_BMS_MODULE_H_

#define BMS_MODULE_NUM_CELLS 5
#define BMS_MODULE_NUM_TEMPERATURE_SENSOR 3

#define BMS_MODULE_VOLTAGE_ARRAY_SIZE BMS_MODULE_NUM_CELLS
#define BMS_MODULE_TEMPERATURE_ARRAY_SIZE BMS_MODULE_NUM_TEMPERATURE_SENSOR
#define BMS_MODULE_SOC_ARRAY_SIZE BMS_MODULE_NUM_CELLS

typedef struct BmsModule {

	/* Public */
	// Retrieves measurements stored in member variables
	void (*get_temperature_array)(struct BmsModule* this, float* temperature_array);
	void (*get_voltage_array)(struct BmsModule* this, float* voltage_array);
	void (*get_soc_array)(struct BmsModule* this, float* soc_array);

	// Accesses hardware and stores the measured values into member variables
	void (*update_temperature)(struct BmsModule* this);
	void (*update_voltage)(struct BmsModule* this);
	// Runs SOC algorithm using soc_calculator and stores the measured values into soc_array
	void (*update_soc)(struct BmsModule* this);

	/* Note: the functions above must be thread safe.
		Ex: get_temperature_array() and update_temperature()
		must work when called from two parallel running threads*/

	/* Private */
	void* _soc_calculator; // pointer to void for now. TODO: Replace with actual object
	float _voltage_array[BMS_MODULE_VOLTAGE_ARRAY_SIZE]; //Voltage of each cell
	float _temperature_array[BMS_MODULE_TEMPERATURE_ARRAY_SIZE];
	float _soc_array[BMS_MODULE_SOC_ARRAY_SIZE];

	SemaphoreHandle_t _temperature_lock;
	SemaphoreHandle_t _voltage_lock;
	SemaphoreHandle_t _soc_lock;

	SPI_HandleTypeDef* _spi_handle;
	GPIO_TypeDef* _spi_cs_port;
	uint16_t _spi_cs_pin;

} BmsModule;

bool bms_module_init(
		BmsModule* this,
		SPI_HandleTypeDef* _spi_handle,
		GPIO_TypeDef* _spi_cs_port,
		uint16_t _spi_cs_pin
		);


#endif /* INC_BMS_MODULE_H_ */
