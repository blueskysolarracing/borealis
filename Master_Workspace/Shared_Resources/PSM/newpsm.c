#include "newpsm.h"


// Helper Functions
void PSM_write(struct PSM_P* PSM, uint8_t address, uint16_t data) {

	char errorMessage[64];
	uint8_t errorMessageLength;

	// First byte should be 6 bit address followed by 0, and 0 (write)
	// Next 2 bytes are the data (all write registers are 16 bit)
	uint8_t buf1 = data >> 8; // 8 MSB
	uint8_t buf2 = data; // 8 LSB
	uint8_t spi_frame[3] = {address<<2, buf1, buf2};

	// Chip select to begin SPI communication
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(PSM->spiInterface, spi_frame, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "(psm_spi_send) ERROR SENDING TO ADDRESS 0x%X WITH DATA 0x%X\r\n", address, data);
		HAL_UART_Transmit(PSM->uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
}

void PSM_read(struct PSM_P* PSM, uint8_t address, uint8_t* buffer, uint8_t numBytes) {
	//variables for error messages
	char errorMessage[64];
	uint8_t errorMessageLength;

	// 6 bit address followed by 0, and 1 (read)
	uint8_t read_req = (address << 2) + 1;

	// Chip select to begin SPI communication
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(PSM->spiInterface, &read_req, 1, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK){
		//successful transmission
		//store received data into buffer
		HAL_SPI_Receive(PSM->spiInterface, buffer, numBytes, MAX_SPI_TRANSMIT_TIMEOUT);

	} else{
		//instruction not sent!
		//transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "ERROR SENDING READ REQUEST TO ADDRESS 0x%X\r\n", address);
		HAL_UART_Transmit(PSM->uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
}


void set_config(struct PSM_P* PSM){

}




void set_read_mode(struct PSM_P* PSM, bool bus_voltage, bool shunt_voltage, bool temp, bool continuous) {
	uint16_t buffer = 0x0;
	if (continuous)
		buffer = 0x8;

	uint8_t* exisitng_settings[2];
	PSM_read(PSM, CONFIG, existing_settings, 2);

	uint16_t old_buffer = existing_settings[1] << 8;
	old_buffer += existing_settings[0];

	old_buffer = old_buffer & 0b111111111111; // Save first 12 old bits

	if (bus_voltage && shunt_voltage && temp) buffer += 0x7;
	else if (bus_voltage && shunt_voltage) buffer += 0x3;
	else if (shunt_voltage && temp) buffer += 0x6;
	else if (bus_voltage && temp) buffer += 0x5;
	else if (shunt_voltage) buffer += 0x2;
	else if (bus_voltage) buffer += 0x1;
	else if (temp) buffer += 0x4;

	// Saves the current avg count
	uint8_t* exisitng_settings[2];
	PSM_read(PSM, CONFIG, existing_settings, 2);

	buffer = buffer << 12; // The buffer is in the last 4 bits of a uint16
	buffer = buffer | old_buffer;

	PSM_write(PSM, CONFIG, buffer);
}


// REFERENCE CHAPTER 8 FOR CALCULATION
void set_temp_cal(struct PSM_P* PSM, uint16_t set) {
	if (set < 0x3FFF) { // only first 14 bits are data
		PSM_write(PSM, SHUNT_TEMP_CAL, set);
	}
}

// REFERENCE CHAPTER 8 FOR CALCULATION
void set_shunt_cal(struct PSM_P* PSM, int setting) {
	if (set < 32767) { // only first 15 bits are data
		PSM_write(PSM, SHUNT_TEMP_CAL, setting);
	}
}

// Returns voltage in nV
float read_shunt_voltage(struct PSM_P* PSM) {
	// Read 3 bytes from VSHUNT
	vbuffer = uint8_t[3];
	PSM_read(PSM, VSHUNT, vbuffer, 3);

	int32_t shuntv = vbuffer[2] << 14 |
		vbuffer[1] << 6 |
		vbuffer[0] >> 2; // bottom 4 bits of register are 0

	abuffer = uint8_t[2];
	PSM_READ(PSM, ADC_CONFIG, abuffer, 2)
	// check bit 4 is high
	if (abuffer[1] & 0x1 != 0) {
		// full range +/- 163.84 mV
		return (float) vbuffer * FULL_SHUNT_RANGE_CONVERSION
	}
	else {
		// limited range +/- 40.96 mV
		return (float) shuntv * LIM_SHUNT_RANGE_CONVERSION
	}
}

// result in uV
float read_bus_voltage(struct PSM_P* PSM) {
	// Read 3 bytes from VBUS
	vbuffer = uint8_t[3];
	PSM_read(PSM, VBUS, vbuffer, 3);

	int32_t busv = vbuffer[2] << 14 |
		vbuffer[1] << 6 |
		vbuffer[0] >> 2; // bottom 4 bits of register are 0

	return (float) busv * VBUS_CONVERSION
}

// Result in m°C
float read_temperature(struct PSM_P* PSM) {
	// Read 2 bytes from DIETEMP
	buffer = uint8_t[2];
	PSM_read(PSM, DIETEMP, buffer, 2);

	int32_t temp = buffer[1] << 8 | buffer[0];

	return (float) temp * DIETEMP_CONVERSION
}

// Result in A
float read_current(struct PSM_P* PSM) {
	// Read 3 bytes from CURRENT
	buffer = uint8_t[3];
	PSM_read(PSM, CURRENT, buffer, 3);

	int32_t current = buffer[2] << 14 |
		buffer[1] << 6 |
		buffer[0] >> 2; // bottom 4 bits of register are 0

	return (float) current * PSM -> current_lsb;
}

// result in W
float read_power(struct PSM_P* PSM) {
	// Read 3 bytes from POWER
	buffer = uint8_t[3];
	PSM_read(PSM, POWER, buffer, 3);

	uint32_t power= buffer[2] << 16 |
		buffer[1] << 8 |
		buffer[0];

	return (float) current * PSM -> current_lsb * POWER_CONVERSION;
}

// result in J
float read_energy(struct PSM_P* PSM) {
	// Read 5 bytes from ENERGY
	buffer = uint8_t[5];
	PSM_read(PSM, ENERGY, buffer, 3);

	uint64_t energy= buffer[4] << 32 |
		buffer[3] << 24 |
		buffer[2] << 16 |
		buffer[1] << 8 |
		buffer[0];

	return (float) current * PSM -> current_lsb * ENERGY_CONVERSION;
}

// result in C
float read_charge(struct PSM_P* PSM) {
	// Read 5 bytes from CHARGE
	buffer = uint8_t[5];
	PSM_read(PSM, CHARGE, buffer, 3);

	int64_t current= buffer[4] << 32 |
		buffer[3] << 24 |
		buffer[2] << 16 |
		buffer[1] << 8 |
		buffer[0];

	return (float) charge * PSM -> current_lsb;

}
void set_adc_averaging_count(struct PSM_P* PSM, uint8_t count) {
	if (count > 0x8)
		return; // Out of bounds

	uint8_t* exisitng_settings[2];
	PSM_read(PSM, CONFIG, existing_settings, 2);

	uint16_t new_settings = existing_settings[1] << 8;
	new_settings += existing_settings[0];

	new_settings = new_settings & 0b1000;
	new_settings += count;
}

void set_conversion_time(struct PSM_P* PSM, uint8_t time) {
	// time is from 50us to 4120us
	// represented by a value of [0, 7]
	// See documentation section 7.6.1.2

	uint8_t* exisitng_settings[2];
	PSM_read(PSM, CONFIG, existing_settings, 2);

	uint16_t new_settings = existing_settings[1] << 8;
	new_settings += existing_settings[0];

	// New Settings now holds the contents of
	// the existing 16 bit settings
	bool bits[3];
	for (int n = 3; n <= 9; n += 3) {
		for (int i = 0; i < 3; i++) {
			// This shifts the three rightmost bits and puts them into the correct spot
			bits[i] = (time & (1 << i)) != 0;
			new_settings = (((new_settings | (1 << n+i)) ^ (1 << n+i))) | (bits[i] << n+i);
		}
	}

	PSM_write(PSM, CONFIG, new_settings);
}




void PSM_init(struct PSM_P* PSM) {
	// Default config. Can rerun these functions after init.
	set_config(psm);
	set_adc_averaging_count(psm, 0x2);
	set_temp_cal(psm, 0x0);
	set_shunt_cal(psm, 0x0);
	set_conversion_time(0x0);
	set_read_mode(psm, true, true, true, true);
}
