/*
 * bms.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */

#include "bms.h"
#include "main.h"

static void get_temperature(Bms* this, float* temperature_array, int bms_module_id);
static void get_voltage(Bms* this, float* voltage_array, int bms_module_id);
static void get_soc(Bms* this, float* soc_array, int bms_module_id);
static void set_current(Bms* this, float current);
static void run(Bms* this);
static void run_thread(void* parameters);
static void measure_with_all_bms_modules(Bms* this);


void bms_init(
		Bms* this,
		SPI_HandleTypeDef* spi_handle,
		GPIO_TypeDef* spi_cs_ports[],
		uint16_t spi_cs_pins[]
) {
	this->get_temperature = get_temperature;
	this->get_voltage = get_voltage;
	this->get_soc = get_soc;
	this->run = run;

	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		bms_module_init(&this->_bms_modules[i], spi_handle, spi_cs_ports[i], spi_cs_pins[i]);
	}
}


static void get_temperature(Bms* this, float* temperature_array, int bms_module_id)
{
	this->_bms_modules[bms_module_id].get_temperature(&this->_bms_modules[bms_module_id], temperature_array);
}

static void get_voltage(Bms* this, float* voltage_array, int bms_module_id)
{
	this->_bms_modules[bms_module_id].get_voltage(&this->_bms_modules[bms_module_id], voltage_array);
}

static void get_soc(Bms* this, float* soc_array, int bms_module_id)
{
	this->_bms_modules[bms_module_id].get_soc(&this->_bms_modules[bms_module_id], soc_array);
}

static void set_current(Bms* this, float current)
{
	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		this->_bms_modules[i].set_current(&this->_bms_modules[i], current);
	}
}

static void run(Bms* this)
{
	// TODO: create thread which calls _measure_all_bms_modules at certain intervals
	BaseType_t status = xTaskCreate(
			run_thread,  //Function that implements the task.
			"BMS run thread",  //Text name for the task.
			5000, 		 //5000 words *4(bytes/word) = 20000 bytes allocated for task's stack
			this,  //Parameter passed into the task.
			4,  //Priority at which the task is created.
			&this->_run_thread_handle  //Used to pass out the created task's handle.
		);
	configASSERT(status == pdPASS);	// Error checking
}

static void run_thread(void* parameters)
{
	Bms* this = (Bms*)parameters;
	while (1) {
		measure_with_all_bms_modules(this);
	}
}

static void measure_with_all_bms_modules(Bms* this)
{
	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		this->_bms_modules[i].measure_temperature(&this->_bms_modules[i]);
		this->_bms_modules[i].measure_voltage(&this->_bms_modules[i]);
		this->_bms_modules[i].compute_soc(&this->_bms_modules[i]);
	}
}
