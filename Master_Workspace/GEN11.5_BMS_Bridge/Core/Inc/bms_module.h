/*
 * bms_module.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */
#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "batteryEKF.h"
#include "stdint.h"


#ifndef INC_BMS_MODULE_H_
#define INC_BMS_MODULE_H_

#define BMS_MODULE_NUM_CELLS NUM_14P_UNITS		// which is 5 except for maybe one Battery Module
#define BMS_MODULE_NUM_TEMPERATURE_SENSOR 3

#define BMS_MODULE_VOLTAGE_ARRAY_SIZE BMS_MODULE_NUM_CELLS
#define BMS_MODULE_TEMPERATURE_ARRAY_SIZE BMS_MODULE_NUM_TEMPERATURE_SENSOR
#define BMS_MODULE_SOC_ARRAY_SIZE BMS_MODULE_NUM_CELLS

typedef struct BmsModule {

	/* Public */
	// Retrieves measurements stored in member variables
	void (*get_temperature)(struct BmsModule* this, float* temperature_array);
	void (*get_voltage)(struct BmsModule* this, float* voltage_array);
	void (*get_soc)(struct BmsModule* this, float* soc_array);

	void (*set_current)(struct BmsModule* this, float current);

	// Accesses hardware and stores the measured values into member variables
	void (*measure_temperature)(struct BmsModule* this);
	void (*measure_voltage)(struct BmsModule* this);
	void (*compute_soc)(struct BmsModule* this);

	/* Note: the functions above must be thread safe.
		Ex: get_temperature() and measure_temperature()
		must work when called from two parallel running threads*/

	/* Private */
	float _voltage_array[BMS_MODULE_VOLTAGE_ARRAY_SIZE]; //Voltage of each cell
	float _temperature_array[BMS_MODULE_TEMPERATURE_ARRAY_SIZE];
	float _soc_array[BMS_MODULE_SOC_ARRAY_SIZE];
	EKF_Model_14p _EKF_models[BMS_MODULE_SOC_ARRAY_SIZE];
	uint32_t _tick_last_soc_compute[BMS_MODULE_SOC_ARRAY_SIZE];
	float _current;
	int _bms_module_id;

	SemaphoreHandle_t _temperature_lock;
	SemaphoreHandle_t _voltage_lock;
	SemaphoreHandle_t _soc_lock;

	SPI_HandleTypeDef* _spi_handle;
	GPIO_TypeDef* _spi_cs_port;
	uint16_t _spi_cs_pin;

} BmsModule;

void bms_module_init(
		BmsModule* this,
		int bms_module_id,
		SPI_HandleTypeDef* _spi_handle,
		GPIO_TypeDef* _spi_cs_port,
		uint16_t _spi_cs_pin
		);



#endif /* INC_BMS_MODULE_H_ */
