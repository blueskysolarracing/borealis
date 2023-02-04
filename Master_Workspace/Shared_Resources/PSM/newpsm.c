#include "newpsm.h"


// Helper Functions
void PSM_write(struct PSM_P* PSM, SPI_HandleTypeDef* spiInterface,
		UART_HandleTypeDef* uartInterface, uint8_t address, uint16_t data) {

	char errorMessage[64];
	uint8_t errorMessageLength;

	// First byte should be 6 bit address followed by 0, and 0 (write)
	// Next 2 bytes are the data (all write registers are 16 bit)
	uint8_t buf1 = data >> 8; // 8 MSB
	uint8_t buf2 = data; // 8 LSB
	uint8_t spi_frame[3] = {address<<2, buf1, buf2};

	// Chip select to begin SPI communication
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET); // USING OLD PSM STRUCT

	if (HAL_SPI_Transmit(spiInterface, spi_frame, 3, MAX_SPI_TRANSMIT_TIMEOUT) != HAL_OK) {
		//data could not be written! transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "(psm_spi_send) ERROR SENDING TO ADDRESS 0x%X WITH DATA 0x%X\r\n", address, data);
		HAL_UART_Transmit(uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}
}

void PSM_read(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface,
		UART_HandleTypeDef* uartInterface, uint8_t address, uint8_t* buffer,
		uint8_t numBytes) {
	//variables for error messages
	char errorMessage[64];
	uint8_t errorMessageLength;

	// 6 bit address followed by 0, and 1 (read)
	uint8_t read_req = (address << 2) + 1;

	// Chip select to begin SPI communication
	HAL_GPIO_WritePin(PSM->CSPort, PSM->CSPin, GPIO_PIN_RESET); // USING OLD PSM STRUCT
	if (HAL_SPI_Transmit(spiInterface, &read_req, 1, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK){
		//successful transmission
		//store received data into buffer
		HAL_SPI_Receive(spiInterface, buffer, numBytes, MAX_SPI_TRANSMIT_TIMEOUT);

	} else{
		//instruction not sent!
		//transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "ERROR SENDING READ REQUEST TO ADDRESS 0x%X\r\n", address);
		HAL_UART_Transmit(uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}

}
