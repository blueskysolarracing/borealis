/*
 * bms.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */

#include "bms.h"
#include "main.h"

static bool get_temperature(Bms* this, float* temperatures, uint8_t bms_module_id, get_mode_t get_mode);
static bool get_voltage(Bms* this, float* voltages, uint8_t bms_module_id, get_mode_t get_mode);
static bool get_state_of_charge(Bms* this, float* state_of_charges, uint8_t bms_module_id);
static void set_current(Bms* this, float current);
static void run_thread(void* parameters);
static void measure_with_all_bms_modules(Bms* this);
static void stop(Bms* this);


void bms_start(
		Bms* this,
		SPI_HandleTypeDef* spi_handle,
		GPIO_TypeDef* spi_cs_ports[],
		uint16_t spi_cs_pins[]
) {
	this->init_flag = 0;
	this->get_temperature = get_temperature;
	this->get_voltage = get_voltage;
	this->get_state_of_charge = get_state_of_charge;
	this->set_current = set_current;
	this->stop = stop;

	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		bms_module_init(&this->_bms_modules[i], i, spi_handle, spi_cs_ports[i], spi_cs_pins[i]);
	}
	this->_run_thread_handle = NULL;

	if (this->_run_thread_handle == NULL) {
		BaseType_t status = xTaskCreate(
				run_thread,  //Function that implements the task.
				"BMS run thread",  //Text name for the task.
				5000, 		 //5000 words *4(bytes/word) bytes allocated for task's stack (note even this size is unnecessary as this thread barely creates any variables)
				this,  //Parameter passed into the task.
				4,  //Priority at which the task is created.
				&this->_run_thread_handle  //Used to pass out the created task's handle.
			);
		configASSERT(status == pdPASS);	// Error checking
	}
}


static bool get_temperature(Bms* this, float* temperatures, uint8_t bms_module_id, get_mode_t get_mode)
{
	if (bms_module_id < BMS_NUM_BMS_MODULES){
		this->_bms_modules[bms_module_id].get_temperature(&this->_bms_modules[bms_module_id], temperatures, get_mode);
		return true;
	} else {
		return false;
	}
}

static bool get_voltage(Bms* this, float* voltages, uint8_t bms_module_id, get_mode_t get_mode)
{
	if (bms_module_id < BMS_NUM_BMS_MODULES){
		this->_bms_modules[bms_module_id].get_voltage(&this->_bms_modules[bms_module_id], voltages, get_mode);
		return true;
	} else {
		return false;
	}
}

static bool get_state_of_charge(Bms* this, float* state_of_charges, uint8_t bms_module_id)
{
	if (bms_module_id < BMS_NUM_BMS_MODULES){
		this->_bms_modules[bms_module_id].get_state_of_charge(&this->_bms_modules[bms_module_id], state_of_charges);
		return true;
	} else {
		return false;
	}
}

static void set_current(Bms* this, float current)
{
	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		if (this->_bms_modules[i].init_flag == 1) {
			this->_bms_modules[i].set_current(&this->_bms_modules[i], current);
		}
	}
}


static void run_thread(void* parameters)
{
	Bms* this = (Bms*)parameters;
	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		while (!this->_bms_modules[i].init_flag) {}
	}
	this->init_flag = 1;
	int i = 0;
	while (1) {
		measure_with_all_bms_modules(this);
		i++;
	}
}

static void measure_with_all_bms_modules(Bms* this)
{
	for (int i = 0; i < BMS_NUM_BMS_MODULES; i++) {
		//this->_bms_modules[i].measure_temperature(&this->_bms_modules[i]);
		this->_bms_modules[i].measure_voltage(&this->_bms_modules[i]);
		this->_bms_modules[i].compute_soc(&this->_bms_modules[i]);
	}
}

static void stop(Bms* this)
{
	if (this->_run_thread_handle != NULL)
		vTaskDelete(this->_run_thread_handle);
	this->_run_thread_handle = NULL;
}
