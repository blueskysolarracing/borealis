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

int adc_range(struct PSM_P* PSM) {
	uint8_t abuffer[2];
	PSM_read(PSM, CONFIG, abuffer, 2);

	// check bit 4 is high
	if ((abuffer[1] & 0x4) != 0) {
		return 1;
	}
	else {
		return 0;
	}
}

// end of helper functions
void set_config(struct PSM_P* PSM){
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort, PSM->LVDSPin, GPIO_PIN_SET);
}

void set_read_mode(struct PSM_P* PSM, uint8_t bus_voltage, uint8_t shunt_voltage, uint8_t temp, uint8_t continuous) {
	uint16_t buffer = 0x0;
	if (continuous)
		buffer = 0x8;

	uint8_t* existing_settings[2];
	PSM_read(PSM, CONFIG, existing_settings, 2);

	uint16_t old_buffer = existing_settings[1];
	old_buffer << 8;
	old_buffer += (uint16_t) existing_settings[0];

	old_buffer = old_buffer & 0xFFF; // Save first 12 old bits

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
	if (setting < 32767) { // only first 15 bits are data
		PSM_write(PSM, SHUNT_TEMP_CAL, setting);
	}
}

// Returns voltage in nV
float read_shunt_voltage(struct PSM_P* PSM) {
	// Read 3 bytes from VSHUNT
	uint8_t vbuffer[3];
	PSM_read(PSM, VSHUNT, vbuffer, 3);

	int32_t shuntv = vbuffer[2] << 14 |
		vbuffer[1] << 6 |
		vbuffer[0] >> 2; // bottom 4 bits of register are 0
	
	uint8_t adc_rangeb = adc_range(PSM);
	if (adc_rangeb == 1) {
		// full range +/- 163.84 mV
		return (float) shuntv * FULL_SHUNT_RANGE_CONVERSION;
	}
	else {
		// limited range +/- 40.96 mV
		return (float) shuntv * LIM_SHUNT_RANGE_CONVERSION;
	}
}

// result in uV
float read_bus_voltage(struct PSM_P* PSM) {
	// Read 3 bytes from VBUS
	uint8_t vbuffer[3];
	PSM_read(PSM, VBUS, vbuffer, 3);

	int32_t busv = vbuffer[2] << 14 |
		vbuffer[1] << 6 |
		vbuffer[0] >> 2; // bottom 4 bits of register are 0

	return (float) busv * VBUS_CONVERSION;
}

// Result in m°C
float read_temperature(struct PSM_P* PSM) {
	// Read 2 bytes from DIETEMP
	uint8_t buffer[2];
	PSM_read(PSM, DIETEMP, buffer, 2);

	int32_t temp = buffer[1] << 8 | buffer[0];

	return (float) temp * DIETEMP_CONVERSION;
}

// Result in A
float read_current(struct PSM_P* PSM) {
	// Read 3 bytes from CURRENT
	uint8_t buffer[3];
	PSM_read(PSM, CURRENT, buffer, 3);

	int32_t current = buffer[2] << 14 |
		buffer[1] << 6 |
		buffer[0] >> 2; // bottom 4 bits of register are 0

	return (float) current * PSM -> current_lsb;
}

// result in W
float read_power(struct PSM_P* PSM) {
	// Read 3 bytes from POWER
	uint8_t buffer[3];
	PSM_read(PSM, POWER, buffer, 3);

	uint32_t power= buffer[2] << 16 |
		buffer[1] << 8 |
		buffer[0];

	return (float) power * PSM -> current_lsb * POWER_CONVERSION;
}

// result in J
float read_energy(struct PSM_P* PSM) {
	// Read 5 bytes from ENERGY
	uint8_t buffer[5];
	PSM_read(PSM, ENERGY, buffer, 3);

	uint64_t energy= buffer[4] << 32 |
		buffer[3] << 24 |
		buffer[2] << 16 |
		buffer[1] << 8 |
		buffer[0];

	return (float) energy * PSM -> current_lsb * ENERGY_CONVERSION;
}

// result in C
float read_charge(struct PSM_P* PSM) {
	// Read 5 bytes from CHARGE
	uint8_t buffer[5];
	PSM_read(PSM, CHARGE, buffer, 3);

	int64_t charge = buffer[4] << 32 |
		buffer[3] << 24 |
		buffer[2] << 16 |
		buffer[1] << 8 |
		buffer[0];

	return (float) charge * PSM -> current_lsb;

}
void set_adc_averaging_count(struct PSM_P* PSM, uint8_t count) {
	if (count > 0x8)
		return; // Out of bounds

	uint8_t* existing_settings[2];
	PSM_read(PSM, CONFIG, existing_settings, 2);

	uint16_t new_settings = existing_settings[1];
	new_settings << 8;
	new_settings += (uint16_t) existing_settings[0];

	new_settings = new_settings & 0b1000;
	new_settings += count;
}

void set_conversion_time(struct PSM_P* PSM, uint8_t time) {
	// time is from 50us to 4120us
	// represented by a value of [0, 7]
	// See documentation section 7.6.1.2

	uint8_t* existing_settings[2];
	PSM_read(PSM, CONFIG, existing_settings, 2);

	uint16_t new_settings = existing_settings[1];
	new_settings << 8;
	new_settings += (uint16_t) existing_settings[0];

	// New Settings now holds the contents of
	// the existing 16 bit settings
	uint8_t bits[3];
	for (int n = 3; n <= 9; n += 3) {
		for (int i = 0; i < 3; i++) {
			// This shifts the three rightmost bits and puts them into the correct spot
			bits[i] = (time & (1 << i)) != 0;
			new_settings = (((new_settings | (1 << n+i)) ^ (1 << (n+i)))) | (bits[i] << (n+i));
		}
	}

	PSM_write(PSM, CONFIG, new_settings);
}

// setting should be overvoltage in uV
void set_shunt_overvoltage_threshold(struct PSM_P* PSM, int setting) {
	uint8_t adc_rangeb = adc_range(PSM);
	int16_t val;
	if (adc_rangeb == 1) {
		val = (int16_t) setting * SHUNT_FULL_OVERVOLTAGE_CONVERSION;
	}
	else {
		val = (int16_t) setting * SHUNT_FULL_OVERVOLTAGE_CONVERSION;
	}

	PSM_write(PSM, SOVL, val);
}

// setting should be undervoltage in uV
void set_shunt_undervoltage_threshold(struct PSM_P* PSM, int setting) {
	uint8_t adc_rangeb = adc_range(PSM);
	int16_t val;
	if (adc_rangeb == 1) {
		val = (int16_t) setting / SHUNT_FULL_OVERVOLTAGE_CONVERSION;
	}
	else {
		val = (int16_t) setting / SHUNT_FULL_OVERVOLTAGE_CONVERSION;
	}

	PSM_write(PSM, SUVL, val);
}

// setting should be overvoltage in mV
void set_bus_overvoltage_threshold(struct PSM_P* PSM, int setting) {
	int16_t val = (int16_t) setting / BUS_OVERVOLTAGE_CONVERSION;
	PSM_write(PSM, BOVL, val);
}

// setting should be overvoltage in mV
void set_bus_undervoltage_threshold(struct PSM_P* PSM, int setting) {
	int16_t val = (int16_t) setting / BUS_OVERVOLTAGE_CONVERSION;
	PSM_write(PSM, BUVL, val);
}

void set_temperature_limit_threshold(struct PSM_P* PSM, int setting) {
	int16_t val = (int16_t) setting / TEMP_LIMIT_CONVERSION;
	PSM_write(PSM, TEMP_LIMIT, val);
}

void set_power_limit_threshold(struct PSM_P* PSM, int setting) {
	int16_t val = (int16_t) setting / POWER_LIMIT_CONVERSION;
	PSM_write(PSM, PWR_LIMIT, val);
}

void set_device_ID(struct PSM_P* PSM, int setting);


void PSM_init(struct PSM_P* PSM) {
	// Default config. Can rerun these functions after init.
	set_config(PSM);
	set_adc_averaging_count(PSM, 0x2);
	set_temp_cal(PSM, 0x0);
	set_shunt_cal(PSM, 0x0);
	set_conversion_time(PSM, 0x0);
	set_read_mode(PSM, 1, 1, 1, 1);
}
