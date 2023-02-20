#ifndef PSM_H__
#define PSM_H__
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"

#define MAX_SPI_TRANSMIT_TIMEOUT 50 //in ms
#define MAX_UART_TRANSMIT_TIMEOUT 50 //in ms
#define PSM_SEND_INTERVAL 250 //250ms between measurements
#define PSM_FIR_FILTER_SAMPLING_FREQ_MCMB 125 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)
#define PSM_FIR_FILTER_SAMPLING_FREQ_BBMB 200 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)
#define PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB 50 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)

// INA229 REGISTERS
#define CONFIG 			0x0
#define ADC_CONFIG		0x1
#define SHUNT_CAL		0x2
#define SHUNT_TEMP_CAL  0x3
#define VSHUNT			0x4
#define VBUS			0x5
#define DIETEMP			0x5
#define CURRENT			0x7
#define POWER			0x8
#define ENERGY			0x9
#define CHARGE			0xA
#define DIAG_ALERT		0xB
#define SOVL			0xC
#define SUVL			0xD
#define BOVL			0xE
#define BUVL			0xF
#define TEMP_LIMIT		0x10
#define PWR_LIMIT		0x11
#define MANUFACTURER_ID	0x3E
#define DEVICE_ID		0x3F

// Conversion Factors
#define VBUS_CONVERSION 195.3125 // uV/LSB
#define FULL_SHUNT_RANGE_CONVERSION 78.125 // nV/LSB
#define LIM_SHUNT_RANGE_CONVERSION 312.5 // nV/LSB
#define DIETEMP_CONVERSION 7.8125 // m°C/LSB
#define POWER_CONVERSION 3.2 // W
#define ENERGY_CONVERSION 16 * POWER_CONVERSION// J
#define SHUNT_FULL_OVERVOLTAGE_CONVERSION 5 // 5uV/LSB
#define SHUNT_LIM_OVERVOLTAGE_CONVERSION 1.25 // 5uV/LSB
#define BUS_OVERVOLTAGE_CONVERSION 3.125 // mV/LSB
#define TEMP_LIMIT_CONVERSION 7.8125 // m°C/LSB
#define POWER_LIMIT_CONVERSION 256*POWER_CONVERSION // 256 * POWERLSB

struct PSM_FIR_Filter {
	float* buf_voltage;
	float* buf_current;

	uint16_t buf_size; //Size of FIFO buffer
	uint16_t live_buf_length_voltage; //Used for averaging (needed because buf will be initially empty)
	uint16_t live_buf_length_current; //Used for averaging (needed because buf will be initially empty)
	uint16_t live_index_voltage; //This tracks the location of the last added voltage
	uint16_t live_index_current; //This tracks the location of the last added current

	float avg_voltage;
	float avg_current;

	void (*push_current) (struct PSM_FIR_Filter* self, float new_value);
	float (*pop_current) (struct PSM_FIR_Filter* self);

	void (*push_voltage) (struct PSM_FIR_Filter* self, float new_value);
	float (*pop_voltage) (struct PSM_FIR_Filter* self);

	float (*get_average) (struct PSM_FIR_Filter* self, uint8_t voltage_or_current);

	void (*push) (struct PSM_FIR_Filter* self, float new_voltage, float new_current);
};

enum measurementType{
	VOLTAGEF,
	CURRENTF
};


struct PSM_P {
	 SPI_HandleTypeDef* spiInterface;
	 UART_HandleTypeDef* uartInterface;

	 GPIO_TypeDef* CSPort;
	 uint16_t CSPin;

	 GPIO_TypeDef* LVDSPort;
	 uint16_t LVDSPin;

	  GPIO_TypeDef* DreadyPort;
	  uint16_t DreadyPin;

	//  float VDCOS;
	//  float CDCOS;

     float current_lsb; // refer to 8.1.2 for calculation
};


// For use in other files
void PSM_init(struct PSM_P* psm);
void PSM_FIR_init(struct PSM_P* psm);
void PSM_update(struct PSM_P* psm, struct PSM_FIR* psm_fir);
void PSM_read_voltage(struct PSM_FIR* psm_fir, float* result);
void PSM_read_current(struct PSM_FIR* psm_fir, float* result);
void PSM_write(struct PSM_P* PSM, uint8_t address, uint16_t data);
void PSM_read(struct PSM_P* PSM, uint8_t address, uint8_t* buffer,
		uint8_t numBytes);

// ----------- HELPER FUNCTIONS FOR CHIP ----------- //
// Register: CONFIG
void reset_psm_power(struct PSM_P* PSM);
void reset_accumulation_reg(struct PSM_P* PSM);

void set_config(struct PSM_P* PSM);
void set_adc_delay(struct PSM_P* PSM); // ms
void set_adc_averaging_count(struct PSM_P* PSM, uint8_t count);
void set_conversion_time(struct PSM_P* PSM, uint8_t time);
void set_temp_cal(struct PSM_P* PSM, uint16_t set);
void set_adc_range(struct PSM_P* PSM, double min, double max);

// Register: ADC_CONFIG
void set_read_mode(struct PSM_P* PSM, uint8_t bus_voltage, uint8_t shunt_voltage, uint8_t temp, uint8_t continuous);
void set_voltage_bus_conversion_time(struct PSM_P* PSM, int setting);
void set_voltage_shunt_conversion_time(struct PSM_P* PSM, int setting);
void set_temp_conversion_time(struct PSM_P* PSM, int setting);
void set_adc_average_count(struct PSM_P* PSM, int setting);

// Register: SHUNT_CAL
void set_shunt_cal(struct PSM_P* PSM, int setting); // TODO FIGURE OUT SETTING VALUE TYPE
void set_shunt_temp_co(struct PSM_P* PSM, int setting); // TODO FIGURE OUT SETTING VALUE TYP

// Measurements and results
float read_shunt_voltage(struct PSM_P* PSM);
float read_bus_voltage(struct PSM_P* PSM);
float read_temperature(struct PSM_P* PSM);
float read_current(struct PSM_P* PSM);
float read_power(struct PSM_P* PSM);
float read_energy(struct PSM_P* PSM);
float read_charge(struct PSM_P* PSM);

// Register: DIAG_ALRT
uint16_t read_diag_alrt(struct PSM_P);

// ALATCH (bit 15) = 1 ->Latched, alert pin and flag will remain active until DIAG_ALRT is read
// ALATCH = 0, Transparent, alert pin and flag will clear when fault is cleared
void set_alert_latch(struct PSM_P* PSM, int setting);
void set_conversion_ready_flag(struct PSM_P* PSM, int setting);
void set_slow_alert_flag(struct PSM_P* PSM, int setting);
void set_alert_polarity(struct PSM_P* PSM, int setting);

// Overflow on bit high, clears on read
int check_energy_overflow(struct PSM_P* PSM);
int check_charge_overflow(struct PSM_P* PSM);
// check_math_overflow is manually cleared by:
// 1. triggering another conversion
// 2. clearing all accumulators with RTSTACC
int check_math_overflow(struct PSM_P* PSM);
int check_temperature_limit_exceeded(struct PSM_P* PSM);
// following are cleared on read when latched mode, error on high
int check_shunt_overvoltage_(struct PSM_P* PSM);
int check_shunt_undervoltage_(struct PSM_P* PSM);
int check_bus_overvoltage(struct PSM_P* PSM);
int check_bus_undervoltage(struct PSM_P* PSM);
int check_power_limit_exceeded(struct PSM_P* PSM);
int check_conversion_completed(struct PSM_P* PSM); // also cleared on new conversion
int check_memory_status(struct PSM_P* PSM); //error on low (for some reason)


// Safety Thresholds
void set_shunt_overvoltage_threshold(struct PSM_P* PSM, int setting);
void set_shunt_undervoltage_threshold(struct PSM_P* PSM, int setting);
void set_bus_overvoltage_threshold(struct PSM_P* PSM, int setting);
void set_bus_undervoltage_threshold(struct PSM_P* PSM, int setting);
void set_temperature_limit_threshold(struct PSM_P* PSM, int setting);
void set_power_limit_threshold(struct PSM_P* PSM, int setting);
void set_device_ID(struct PSM_P* PSM, int setting);

#endif
