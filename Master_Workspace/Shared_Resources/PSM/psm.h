#ifndef PSM_H__
#define PSM_H__

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"

#define MAX_SPI_TRANSMIT_TIMEOUT 50 //in ms
#define MAX_UART_TRANSMIT_TIMEOUT 50 //in ms
#define NUM_AVG 1 //Number of averages to take for each measurements

/*
 * With 781.25kHz, PSMRead takes 1.368ms.
 * With 1.5625MHz, PSMRead takes 1.156ms.
 * With 3.125MHz, PSMRead takes 1.060ms. (Optimized to 0.401ms)
 */

//ADE7912 register addresses; names identical to datasheet
#define IWV 			0x0
#define V1WV 			0x1
#define V2WV 			0x2
#define ADC_CRC 		0x4
#define CTRL_CRC 		0x5
#define CNT_SNAPSHOT 	0x7
#define CONFIG 			0x8
#define STATUS0 		0x9
#define LOCK_ADDRESS 	0xA
#define SYNC_SNAP 		0xB
#define COUNTER0 		0xC
#define COUNTER1 		0xD
#define EMI_CTRL 		0xE
#define STATUS1 		0xF
#define TEMPOS 			0x18

//ADE7912 instructions
#define UNLOCK_COMMAND 	0x9C
#define LOCK_COMMAND 	0xCA

#define PWR_DWN_ENABLE 0 //Set to 1 to shutdown ADE7912s between measurements (WILL ADD ~100ms DELAY AS THE ISOLATED CONVERTERS NEED TO TURN ON). Saves ~10mA @ 3.3V
#define WRITE_PROTECTION_ENABLE 1 //Set to 1 to disable write protection between reads/writes (saves time)
#define PSM_SEND_INTERVAL 250 //250ms between measurements
#define PSM_FIR_FILTER_SAMPLING_FREQ_MCMB 125 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)
#define PSM_FIR_FILTER_SAMPLING_FREQ_BBMB 125 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)
#define PSM_FIR_FILTER_SAMPLING_FREQ_PPTMB 50 //Number of samples contained in FIR filter FIFO buffer (and sampling frequency in Hz)

/* FIR averaging filter struct
 * The PSM library makes available the struct and functions needed to implement a FIFO buffer-based FIR averaging filter.
 * The user is expected to periodically push new values into the buffer. The buffer's average can be read asynchronously.
 */
struct PSM_FIR_Filter {
	//--- VARIABLES ---//
	float* buf_voltage;
	float* buf_current;

	uint16_t buf_size; //Size of FIFO buffer
	uint16_t live_buf_length_voltage; //Used for averaging (needed because buf will be initially empty)
	uint16_t live_buf_length_current; //Used for averaging (needed because buf will be initially empty)
	uint16_t live_index_voltage; //This tracks the location of the last added voltage
	uint16_t live_index_current; //This tracks the location of the last added current

	float avg_voltage; //Output of the filter
	float avg_current; //Output of the filter

	//--- API ---//
	void (*push) (struct PSM_FIR_Filter* self, float new_value, uint8_t voltage_or_current);
	float (*pop) (struct PSM_FIR_Filter* self, uint8_t voltage_or_current);
	float (*get_average) (struct PSM_FIR_Filter* self, uint8_t voltage_or_current);
};

enum measurementType{
	VOLTAGE,
	CURRENT
};
/*constants for calibrating voltage and current measurements
These belong to the PSM in different boxes. IDs are written on the PSM.
CDCOS_CHx = "Current DC OffSet" of PSM channel x
VDCOS_CHx = "Voltage DC OffSet" of PSM channel x
CM_CHx = "Current Multiplier" of PSM channel x
VM_CHx = "Voltage Multiplier" of PSM channel x

Nominally, the voltage multipler is (1.2) * (1 / (2^23 - 1)) * (1 + 664*(1/R5 + 1/480)) [output in V; R5 in kR]
Nominally, the currrent multiplier is (1.2) * (1 / (2^23 - 1)) / R8 [output in A; R8 in mR]
*/

struct PSM_Peripheral{
	//SPI ports and GPIOs for PSM. Varies for each motherboard. To be initialized in main.c
	GPIO_TypeDef* CSPort0;
	uint16_t CSPin0;

	GPIO_TypeDef* CSPort1;
	uint16_t CSPin1;

	GPIO_TypeDef* CSPort2;
	uint16_t CSPin2;

	GPIO_TypeDef* CSPort3;
	uint16_t CSPin3;

	GPIO_TypeDef* LVDSPort;
	uint16_t LVDSPin;

	GPIO_TypeDef* DreadyPort;
	uint16_t DreadyPin;

	//Constants for each channel
	float VDCOS_CH1;
	float CDCOS_CH1;
	float VM_CH1;
	float CM_CH1;

	float VDCOS_CH2;
	float CDCOS_CH2;
	float VM_CH2;
	float CM_CH2;

	float VDCOS_CH3;
	float CDCOS_CH3;
	float VM_CH3;
	float CM_CH3;

	float VDCOS_CH4;
	float CDCOS_CH4;
	float VM_CH4;
	float CM_CH4;
};

//------ FUNCTION PROTOTYPES ------//
double arrayToDouble(uint8_t* aryPtr, uint8_t size);
void PSM_Init(struct PSM_Peripheral* PSM, uint8_t PSM_ID);
void PSM_FIR_Init(struct PSM_FIR_Filter* filter);
void writeOnePSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t address, uint8_t data, uint8_t channelNumber);
void writeMultiplePSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t address, uint8_t data,
		uint8_t EN_c1, uint8_t EN_c2, uint8_t EN_c3, uint8_t EN_c4);
void readFromPSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t address, uint8_t* buffer, uint16_t numBytes, uint8_t channelNumber);
int configPSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, char* channels, uint32_t timeout);
void PSMRead(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t CLKOUT, uint8_t masterPSM, uint8_t channelNumber, double dataOut[], uint8_t dataOutLen);
void PSMReadTemperature(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t masterPSM);
void PSMCalib(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, double voltageToInputRatio,
		double shuntResistance, uint8_t masterPSM, uint8_t channelNumber);
float PSM_FIR_pop(struct PSM_FIR_Filter* filter, uint8_t voltage_or_current);
void PSM_FIR_push(struct PSM_FIR_Filter* filter, float new_val, uint8_t voltage_or_current);
float PSM_FIR_get_average(struct PSM_FIR_Filter* filter, uint8_t voltage_or_current);
#endif
