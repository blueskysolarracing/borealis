#ifndef PSM_H__
#define PSM_H__
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"

#define MAX_SPI_TRANSMIT_TIMEOUT 50 //in ms
#define MAX_UART_TRANSMIT_TIMEOUT 50 //in ms

// To get rid of
#define PWR_DWN_ENABLE 0 //Set to 1 to shutdown ADE7912s between measurements (WILL ADD ~100ms DELAY AS THE ISOLATED CONVERTERS NEED TO TURN ON). Saves ~10mA @ 3.3V
#define WRITE_PROTECTION_ENABLE 1 //Set to 1 to disable write protection between reads/writes (saves time)
#define PSM_SEND_INTERVAL 250 //250ms between measurements
#define PSM_FIR_FILTER_SAMPLING_FREQ_MCMB 125 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)
#define PSM_FIR_FILTER_SAMPLING_FREQ_BBMB 200 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)
#define PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB 50 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)

// INA229 REGISTERS
#define CONFIG 				0x0
#define ADC_CONFIG			0x1
#define SHUNT_CAL			0x2
#define SHUNT_TEMP_CAL  	0x3
#define VSHUNT				0x4
#define VBUS				0x5
#define DIETEMP				0x5
#define CURRENT				0x7
#define POWER				0x8
#define ENERGY				0x9
#define CHARGE				0xA
#define DIAG_ALERT			0xB
#define SOVL				0xC
#define SUVL				0xD
#define BOVL				0xE
#define BUVL				0xF
#define TEMP_LIMIT			0x10
#define PWR_LIMIT			0x11
#define MANUFACTURER_ID		0x3E
#define DEVICE_ID			0x3F

// Design constants
#define MAX_CURRENT 60
#define SHUNT_RESISTANCE 0.002

// Conversion Factors
#define VBUS_CONVERSION 						(195.3125 * 0.000001)
#define CURRENT_CONVERSION(maxCurrent) 			((double)(maxCurrent)/(double)524288)
#define LIMITED_SHUNT_RANGE_CONVERSION 			(78.125 * 0.000000001)
#define FULL_SHUNT_RANGE_CONVERSION 			(312.5 * 0.000000001)
#define DIETEMP_CONVERSION 						(7.8125 * 0.001)
#define POWER_CONVERSION 						(3.2 * CURRENT_CONVERSION(MAX_CURRENT))
#define ENERGY_CONVERSION 						(16 * POWER_CONVERSION * CURRENT_CONVERSION(MAX_CURRENT))

#define FULL_SHUNT_VOLTAGE_CONVERSION 			(5 * 0.000001)
#define LIM_SHUNT_VOLTAGE_CONVERSION 			(1.25 * 0.000001)
#define VBUS_THRESHOLD_CONVERSION 				(3.125 * 0.001)
#define TEMP_LIMIT_CONVERSION 					(7.8125 * 0.001)
#define POWER_LIMIT_CONVERSION 					(256*POWER_CONVERSION)

struct PSM_P {
	 SPI_HandleTypeDef* spi_handle;
	 UART_HandleTypeDef* uart_handle;

	 GPIO_TypeDef* CSPort;
	 uint16_t CSPin;

	 GPIO_TypeDef* LVDSPort;
	 uint16_t LVDSPin;
};

//// Register: DIAG_ALRT
////uint16_t read_diag_alrt(struct PSM_P);
//
//// ALATCH (bit 15) = 1 ->Latched, alert pin and flag will remain active until DIAG_ALRT is read
//// ALATCH = 0, Transparent, alert pin and flag will clear when fault is cleared
//void set_alert_latch(struct PSM_P* PSM, int setting);
//void set_conversion_ready_flag(struct PSM_P* PSM, int setting);
//void set_slow_alert_flag(struct PSM_P* PSM, int setting);
//void set_alert_polarity(struct PSM_P* PSM, int setting);
//
//// Overflow on bit high, clears on read
//int check_energy_overflow(struct PSM_P* PSM);
//int check_charge_overflow(struct PSM_P* PSM);
//// check_math_overflow is manually cleared by:
//// 1. triggering another conversion
//// 2. clearing all accumulators with RTSTACC
//int check_math_overflow(struct PSM_P* PSM);
//int check_temperature_limit_exceeded(struct PSM_P* PSM);
//// following are cleared on read when latched mode, error on high
//int check_shunt_overvoltage_(struct PSM_P* PSM);
//int check_shunt_undervoltage_(struct PSM_P* PSM);
//int check_bus_overvoltage(struct PSM_P* PSM);
//int check_bus_undervoltage(struct PSM_P* PSM);
//int check_power_limit_exceeded(struct PSM_P* PSM);
//int check_conversion_completed(struct PSM_P* PSM); // also cleared on new conversion
//int check_memory_status(struct PSM_P* PSM); //error on low (for some reason)


// Configuration functions
void PSM_init(struct PSM_P * PSM, SPI_HandleTypeDef* spi_handle, UART_HandleTypeDef* uart_handle);
void config_PSM(struct PSM_P * PSM);
void resetPSM(struct PSM_P * PSM);

// Function used for initial testing (reading and writing to registers)
void test_config(struct PSM_P* PSM, SPI_HandleTypeDef* spi_handle, UART_HandleTypeDef* uart_handle);

// Read Data
float readPSM(struct PSM_P * PSM, uint8_t addr, uint8_t numBytes);

// Safety Thresholds
void writeOnce_PSM(struct PSM_P * PSM, uint8_t * data, uint8_t numBytes);
void set_shunt_overvoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold);
void set_shunt_undervoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold);
void set_bus_overvoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold);
void set_bus_undervoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold);
void set_power_limit_threshold(struct PSM_P* PSM, uint16_t power_threshold);
void set_device_ID(struct PSM_P* PSM, uint16_t deviceID);


#endif
