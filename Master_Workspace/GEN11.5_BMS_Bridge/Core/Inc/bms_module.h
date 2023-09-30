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
#include "cQueue.h"
#include "battery_config.h"

#ifndef INC_BMS_MODULE_H_
#define INC_BMS_MODULE_H_

#define BMS_MODULE_NUM_TEMPERATURE_SENSOR NUM_TEMP_SENSORS_PER_MODULE

#define BMS_MODULE_NUM_VOLTAGES NUM_CELLS_PER_MODULE
#define BMS_MODULE_NUM_TEMPERATURES BMS_MODULE_NUM_TEMPERATURE_SENSOR
#define BMS_MODULE_NUM_STATE_OF_CHARGES NUM_CELLS_PER_MODULE

#define STATIC_FLOAT_QUEUE_NUM_VALUES 20

typedef enum {
	GET_MOST_RECENT,
	GET_PAST_AVERAGE,
	GET_FILTERED_RESULT
} get_mode_t;

typedef struct StaticFloatQueue {
	float vals[STATIC_FLOAT_QUEUE_NUM_VALUES];
	Queue_t q;
}StaticFloatQueue;

typedef struct BmsModule {

	/* Public */
	// Retrieves measurements stored in member variables
	void (*get_temperature)(struct BmsModule* this, float* temperatures, get_mode_t get_mode);
	void (*get_voltage)(struct BmsModule* this, float* voltages, get_mode_t get_mode);
	void (*get_state_of_charge)(struct BmsModule* this, float* state_of_charges);

	void (*set_current)(struct BmsModule* this, float current);

	// Accesses hardware and stores the measured values into member variables
	void (*measure_temperature)(struct BmsModule* this);
	void (*measure_voltage)(struct BmsModule* this);
	void (*compute_soc)(struct BmsModule* this);

	/* Note: the functions above must be thread safe.
		Ex: get_temperature() and measure_temperature()
		must work when called from two parallel running threads*/

	/* Private */
	float _voltages[BMS_MODULE_NUM_VOLTAGES]; //Voltage of each cell
	StaticFloatQueue past_voltages[BMS_MODULE_NUM_VOLTAGES];

	float _temperatures[BMS_MODULE_NUM_TEMPERATURES];
	StaticFloatQueue past_temperatures[BMS_MODULE_NUM_TEMPERATURES];

	float _state_of_charges[BMS_MODULE_NUM_STATE_OF_CHARGES];
	EKF_Model_14p _EKF_models[BMS_MODULE_NUM_STATE_OF_CHARGES];
	uint32_t _tick_last_soc_compute[BMS_MODULE_NUM_STATE_OF_CHARGES];
	float _current;
	int _bms_module_id;

	SemaphoreHandle_t _data_lock;

	SPI_HandleTypeDef* _spi_handle;
	GPIO_TypeDef* _spi_cs_port;
	uint16_t _spi_cs_pin;

	int init_flag;// raise after init function finishes, so freertos no issue

} BmsModule;

void bms_module_init(
	BmsModule* this,
	int bms_module_id,
	SPI_HandleTypeDef* _spi_handle,
	GPIO_TypeDef* _spi_cs_port,
	uint16_t _spi_cs_pin
);

// Testing function
void bms_module_while_loop_test(void* parameters);



#endif /* INC_BMS_MODULE_H_ */
