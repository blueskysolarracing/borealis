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
	void (*get_temperature)(struct BmsModule* this, float* temperature_array);
	void (*get_voltage)(struct BmsModule* this, float* voltage_array);
	void (*get_soc)(struct BmsModule* this, float* soc_array);

	// Accesses hardware and stores the measured values into member variables
	void (*measure_temperature)(struct BmsModule* this);
	void (*measure_voltage)(struct BmsModule* this);
	void (*compute_soc)(struct BmsModule* this);

	/* Note: the functions above must be thread safe.
		Ex: get_temperature_array() and measure_temperature()
		must work when called from two parallel running threads*/

	/* Private */
	void* _soc_algorithm; // pointer to void for now. TODO: Replace with actual object
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

void bms_module_init(
		BmsModule* this,
		SPI_HandleTypeDef* _spi_handle,
		GPIO_TypeDef* _spi_cs_port,
		uint16_t _spi_cs_pin
		);

#include "stdint.h"

void LTC6810GeneratePECbits(int CMD0[],int CMD1[], int PCE0[], int PCE1[]);
void LTC6810GeneratePECbits6Byte(int data6Byte[], int PCE0[], int PCE1[]);
uint8_t LTC6810ArrayToByte(int arrayIn[]); //8 bit array to a byte
void LTC6810CommandToArray(int command, int CMD0ref[], int CMD1ref[]);
void LTC6810CommandGenerate(int command, uint8_t dataToSend[]); //dataToSend is array pass by reference
int LTC6810InitializeAndCheck();
int LTC6810VoltageDataConversion(uint8_t lowByte, uint8_t highByte);
void LTC6810Init(BmsModule* this, int GPIO4, int GPIO3, int GPIO2 ,int DCC5, int DCC4, int DCC3, int DCC2, int DCC1);


#endif /* INC_BMS_MODULE_H_ */
