#include "newpsm.h"

void PSM_init(struct PSM_P* PSM, SPI_HandleTypeDef* spi_handle, UART_HandleTypeDef* uart_handle){

	PSM->spi_handle = spi_handle;
	PSM->uart_handle = uart_handle;

	config_PSM(PSM);

}

void config_PSM(struct PSM_P* PSM){

	/* Current configuration settings:
	 * ADCRANGE: +/-163.84 mV
	 * Continous bus voltage, shunt voltage
	 * Bus Voltage Conversion Time: 540us
	 * Shunt Voltage Conversion Time: 540us
	 * Temperature Conversion Time: 540us
	 * ADC Sample Averaging Count: 16
	 */

	// Write documentation for config settings later - Tony

	uint8_t config_buffer[3] = {(CONFIG << 2), 0b00000000, 0b01000000};
	uint8_t adc_config_buffer[3] = {(ADC_CONFIG << 2), 0b10111001, 0b00100010};
	uint8_t shunt_cal_buffer[3] = {(SHUNT_CAL << 2), 0b00001001, 0b11000100}; // Max current set as 50A
	//uint8_t shunt_cal_buffer[3] = {(SHUNT_CAL << 2), 0b00001011, 0b10111000}; // Max current set as 60A

	char errorMessage[64];
	uint8_t errorMessageLength;

	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	if (HAL_SPI_Transmit(PSM->spi_handle, config_buffer, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "ERROR SENDING TO CONFIG");
		HAL_UART_Transmit(PSM->uart_handle, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}
	if (HAL_SPI_Transmit(PSM->spi_handle, adc_config_buffer, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "ERROR SENDING TO ADC CONFIG");
		HAL_UART_Transmit(PSM->uart_handle, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}
	if (HAL_SPI_Transmit(PSM->spi_handle, shunt_cal_buffer, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "ERROR SENDING TO SHUNT CAL");
		HAL_UART_Transmit(PSM->uart_handle, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}

	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);

}

void resetPSM(struct PSM_P * PSM){
	// not implemented, for device reset
	HAL_Delay(0);
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	uint8_t buffer[3] = {0x0, 0b10000000, 0b00000000}; // Device Reset Settings
	if (HAL_SPI_Transmit(PSM->spi_handle, buffer, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		while(1){
			// just idle here
		}
	}

	HAL_Delay(0);
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);

}

float readPSM(struct PSM_P * PSM, uint8_t addr, uint8_t numBytes){

	HAL_Delay(0);
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	uint8_t reg_addr[1] = {((addr << 2) + 1)};
	uint8_t buffer[numBytes];

	for(uint8_t i = 0; i < numBytes; i++){
		buffer[i] = 0;
	}

	if (HAL_SPI_Transmit(PSM->spi_handle, reg_addr, 1, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK){
		HAL_SPI_Receive(PSM->spi_handle, buffer, 3, MAX_SPI_TRANSMIT_TIMEOUT);
	}

	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);

	uint64_t raw_data;
	if(numBytes == 3){
		raw_data = ((uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | buffer[2]);
	}
	else if(numBytes == 5){ // energy and charge registers
		raw_data = ((uint64_t)buffer[0] << 32 | (uint64_t)buffer[1] << 24 | (uint32_t)buffer[2] << 16 |
				(uint16_t)buffer[3] << 8 | buffer[4]);
	}
	else{
		raw_data = -1; // invalid
		return raw_data;
	}

	float result;
	switch(addr){
		case VBUS:
			result = (raw_data >> 4) * VBUS_CONVERSION / VOLTAGE_DIVIDER;
			break;

		case CURRENT:
			result = (CURRENT_CONVERSION(MAX_CURRENT) * (raw_data >> 4)) * (1-CURRENT_ERROR_MULTIPLIER) - CURRENT_ERROR_OFFSET;
			break;

		case POWER:
			result = raw_data * POWER_CONVERSION;
			break;

		case VSHUNT:
			result  = (raw_data >> 4) * FULL_SHUNT_RANGE_CONVERSION;
			break;

		case ENERGY:
			result = raw_data * ENERGY_CONVERSION;
			break;

		case CHARGE:
			result = raw_data * CURRENT_CONVERSION(MAX_CURRENT);
			break;
	}
	return result;
}

void writeOnce_PSM(struct PSM_P * PSM, uint8_t* data, uint8_t numBytes){

	char errorMessage[64];
	uint8_t errorMessageLength;
	HAL_Delay(1);

	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	if (HAL_SPI_Transmit(PSM->spi_handle, data, numBytes, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "ERROR SENDING TO CONFIG");
		HAL_UART_Transmit(PSM->uart_handle, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);

}

void set_shunt_overvoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold){
	voltage_threshold = voltage_threshold * FULL_SHUNT_VOLTAGE_CONVERSION;
	uint8_t buffer[3] = {(SOVL << 2), (voltage_threshold >> 8), (voltage_threshold & 0xFF)};
	writeOnce_PSM(PSM, buffer, 3);
}

void set_shunt_undervoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold){
	voltage_threshold = voltage_threshold * FULL_SHUNT_VOLTAGE_CONVERSION;
	uint8_t buffer[3] = {(SUVL << 2), (voltage_threshold >> 8), (voltage_threshold & 0xFF)};
	writeOnce_PSM(PSM, buffer, 3);
}

void set_bus_overvoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold){
	voltage_threshold = voltage_threshold * VBUS_THRESHOLD_CONVERSION;
	uint8_t buffer[3] = {(SOVL << 2), ((voltage_threshold >> 8) & ~(1 << 7)), (voltage_threshold & 0xFF)};
	writeOnce_PSM(PSM, buffer, 3);
}

void set_bus_undervoltage_threshold(struct PSM_P* PSM, uint16_t voltage_threshold){
	voltage_threshold = voltage_threshold * VBUS_THRESHOLD_CONVERSION;
	uint8_t buffer[3] = {(SOVL << 2), ((voltage_threshold >> 8) & ~(1 << 7)), (voltage_threshold & 0xFF)};
	writeOnce_PSM(PSM, buffer, 3);
}

void set_power_limit_threshold(struct PSM_P* PSM, uint16_t power_threshold){
	power_threshold = power_threshold * POWER_LIMIT_CONVERSION;
	uint8_t buffer[3] = {(SOVL << 2), (power_threshold >> 8), (power_threshold & 0xFF)};
	writeOnce_PSM(PSM, buffer, 3);
}

void set_device_ID(struct PSM_P* PSM, uint16_t deviceID){
	uint8_t buffer[3] = {(SOVL << 2), (deviceID >> 4), ((deviceID & 0xF) << 4)};
	writeOnce_PSM(PSM, buffer, 3);
}


// Test function to confirm config settings
void test_config(struct PSM_P* PSM, SPI_HandleTypeDef* spi_handle, UART_HandleTypeDef* uart_handle){

	PSM->spi_handle = spi_handle;
	PSM->uart_handle = uart_handle;


	// config register
	uint8_t spi_frame[3] = {0x0, 0b00000000, 0b01000000}; // default settings for config register
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);
	//HAL_Delay(0);

	if (HAL_SPI_Transmit(PSM->spi_handle, spi_frame, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		while(1){
			// just idle here
		}
	}
	spi_frame[0]++;
	uint8_t readbuf1[2];
	if (HAL_SPI_Transmit(PSM->spi_handle, spi_frame, 1, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK) {
		HAL_SPI_Receive(PSM->spi_handle, readbuf1, 2, MAX_SPI_TRANSMIT_TIMEOUT);
	}

	// adc config register
	uint8_t spi_frame1[3] = {(0x1 << 2), 0b11111001, 0b00100010};
	if (HAL_SPI_Transmit(PSM->spi_handle, spi_frame1, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		while(1){
			// just idle here
		}
	}

	spi_frame1[0]++;
	uint8_t readbuf2[2];
	if (HAL_SPI_Transmit(PSM->spi_handle, spi_frame1, 1, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK) {
		HAL_SPI_Receive(PSM->spi_handle, readbuf2, 2, MAX_SPI_TRANSMIT_TIMEOUT);
	}

	// shunt_cal register
	// adcrange = 0 is +/-163.84mV range; acdrange = 1 is +/-40.96mV range
	uint8_t spi_frame2[3] = {(0x2 << 2), 0b00001001, 0b11000100};
	if (HAL_SPI_Transmit(PSM->spi_handle, spi_frame2, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		while(1){
			// just idle here
		}
	}
	uint8_t readbuf3[2];
	spi_frame2[0]++;
	if (HAL_SPI_Transmit(PSM->spi_handle, spi_frame2, 1, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK) {
		HAL_SPI_Receive(PSM->spi_handle, readbuf3, 2, MAX_SPI_TRANSMIT_TIMEOUT);
	}

	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);

}

// FIR FILTER IMPLEMENTATION

float PSM_FIR_get_average(struct PSM_FIR_Filter* filter, uint8_t voltage_or_current){
	if (voltage_or_current == VOLTAGE_MEASUREMENT){
		if (filter->live_buf_length_voltage == 0) return 0; //If no data has been added to the buffer

		float accumulator = 0;

		for (int i = 0; i < filter->live_buf_length_voltage; i++){
			accumulator += filter->buf_voltage[i];
		}

		float average = accumulator / filter->live_buf_length_voltage;
		filter->avg_voltage = average;
		return average;

	} else if (voltage_or_current == CURRENT_MEASUREMENT) {

		float accumulator = 0;

		for (int i = 0; i < filter->live_buf_length_current; i++){
			accumulator += filter->buf_current[i];
		}

		float average = accumulator / filter->live_buf_length_current;
		filter->avg_current = average;
		return average;
	}
}

float PSM_FIR_pop(struct PSM_FIR_Filter* filter, uint8_t voltage_or_current){
	if (voltage_or_current == VOLTAGE_MEASUREMENT){
		if (filter->live_buf_length_voltage == 0) return 0; //If empty

		if (filter->live_index_voltage == 0){ //Wrap around if the last added value is at index 0
			filter->live_index_voltage = filter->buf_size - 1;
			return filter->buf_voltage[0];
		} else {
			filter->live_index_voltage--;
			return filter->buf_voltage[filter->live_index_voltage + 1];
		}

	} else if (voltage_or_current == CURRENT_MEASUREMENT) {
		if (filter->live_buf_length_current == 0) return 0; //If empty

		if (filter->live_index_current == 0){ //Wrap around if the last added value is at index 0
			filter->live_index_current = filter->buf_size - 1;
			return filter->buf_current[0];
		} else {
			filter->live_index_current--;
			return filter->buf_voltage[filter->live_index_current + 1];
		}
	}
}

void PSM_FIR_push(struct PSM_FIR_Filter* filter, float new_val, uint8_t voltage_or_current){
	if (voltage_or_current == VOLTAGE_MEASUREMENT){
		 //Tracks live content as buffer fills up after startup
		if (filter->live_buf_length_voltage < filter->buf_size) filter->live_buf_length_voltage++;

		 //Wrap around circular buffer
		if (filter->live_index_voltage == filter->buf_size - 1)	filter->live_index_voltage = 0;
		else filter->live_index_voltage++;

		//Add new value
		filter->buf_voltage[filter->live_index_voltage] = new_val;

	} else if (voltage_or_current == CURRENT_MEASUREMENT){
		 //Tracks live content as buffer fills up after startup
		if (filter->live_buf_length_current < filter->buf_size) filter->live_buf_length_current++;

		 //Wrap around circular buffer
		if (filter->live_index_current == filter->buf_size - 1)	filter->live_index_current = 0;
		else filter->live_index_current++;

		//Add new value
		filter->buf_current[filter->live_index_current] = new_val;
	}
}

void PSM_FIR_Init(struct PSM_FIR_Filter* filter){
	filter->get_average = PSM_FIR_get_average;
	filter->pop = PSM_FIR_pop;
	filter->push = PSM_FIR_push;

	filter->live_index_voltage = 0;
	filter->live_index_current = 0;

	filter->live_buf_length_voltage = 0;
	filter->live_buf_length_current = 0;
}

