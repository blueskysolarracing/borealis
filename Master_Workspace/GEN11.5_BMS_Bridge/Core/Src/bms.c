/*
 * bms.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */

#include "bms.h"
#include "main.h"

static void get_temperature(Bms* this, float* temperatures, int bms_module_id);
static void get_voltage(Bms* this, float* voltages, int bms_module_id);
static void get_state_of_charge(Bms* this, float* state_of_charges, int bms_module_id);
static void set_current(Bms* this, float current);
static void run(Bms* this);
static void run_thread(void* parameters);
static void measure_with_all_bms_modules(Bms* this);
static void stop(Bms* this);


void bms_init(
		Bms* this,
		SPI_HandleTypeDef* spi_handle,
		GPIO_TypeDef* spi_cs_ports[],
		uint16_t spi_cs_pins[]
) {
	this->get_temperature = get_temperature;
	this->get_voltage = get_voltage;
	this->get_state_of_charge = get_state_of_charge;
	this->set_current = set_current;
	this->run = run;
	this->stop = stop;

	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		bms_module_init(&this->_bms_modules[i], i, spi_handle, spi_cs_ports[i], spi_cs_pins[i]);
	}
	this->_run_thread_handle = NULL;
}


static void get_temperature(Bms* this, float* temperatures, int bms_module_id)
{
	this->_bms_modules[bms_module_id].get_temperature(&this->_bms_modules[bms_module_id], temperatures);
}

static void get_voltage(Bms* this, float* voltages, int bms_module_id)
{
	this->_bms_modules[bms_module_id].get_voltage(&this->_bms_modules[bms_module_id], voltages);
}

static void get_state_of_charge(Bms* this, float* state_of_charges, int bms_module_id)
{
	this->_bms_modules[bms_module_id].get_state_of_charge(&this->_bms_modules[bms_module_id], state_of_charges);
}

static void set_current(Bms* this, float current)
{
	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		this->_bms_modules[i].set_current(&this->_bms_modules[i], current);
	}
}

static void run(Bms* this)
{
	if (this->_run_thread_handle == NULL) {
		BaseType_t status = xTaskCreate(
				run_thread,  //Function that implements the task.
				"BMS run thread",  //Text name for the task.
				256, 		 //256 words *4(bytes/word) = 1024 bytes allocated for task's stack (note even this size is unnecessary as this thread barely creates any variables)
				this,  //Parameter passed into the task.
				4,  //Priority at which the task is created.
				&this->_run_thread_handle  //Used to pass out the created task's handle.
			);
		configASSERT(status == pdPASS);	// Error checking
	}
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

static void stop(Bms* this)
{
	if (this->_run_thread_handle != NULL)
		vTaskDelete(this->_run_thread_handle);
}
