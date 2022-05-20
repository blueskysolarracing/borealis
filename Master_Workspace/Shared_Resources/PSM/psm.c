#include "psm.h"

// Helper functions that programmer could use
// converts elements in an array back to double
double arrayToDouble(uint8_t* aryPtr, uint8_t size) {
	double val = 0;
	uint8_t* valPtr = (uint8_t*)&val + sizeof(val)-1;

	for (int i = size-1; i >= 0 && valPtr >= (uint8_t*)&val; i--) {
		*valPtr = aryPtr[i];
		valPtr--;
	}
	return val;
}
// writes elements in double into individual elements in an array
void doubleToArray(double val, uint8_t* aryPtr) {
    uint8_t aryIdx = 0;
    uint8_t* ptr = (uint8_t*)&val;
    for(; aryIdx<sizeof(val); aryIdx++){
    	aryPtr[aryIdx] = *ptr;
    	ptr++;
    }
}

//PSM_Init()
void PSM_Init(struct PSM_Peripheral* PSM, uint8_t PSM_ID){
	// Set all chip select pins to 1 to disable SPI transmission
	HAL_GPIO_WritePin(PSM->CSPort0, PSM->CSPin0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort1, PSM->CSPin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort2, PSM->CSPin2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort3, PSM->CSPin3, GPIO_PIN_SET);

	// Set LVDS to disabled by outputting logic low
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);

	PSM->VDCOS_CH1 = 0;
	PSM->CDCOS_CH1 = 0;
	PSM->VM_CH1 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
	PSM->CM_CH1 = (1.2) / 8388607 / 0.5; //0.5mR;

	//Load in calibrated parameters
	switch (PSM_ID){
		case 1: //PSM in battery box
			PSM->VDCOS_CH1 = 354159;
			PSM->CDCOS_CH1 = 325934;
			PSM->VM_CH1 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
			PSM->CM_CH1 = (1.2) / 8388607 / 4; //4mR

			PSM->VDCOS_CH2 = 354159;
			PSM->CDCOS_CH2 = 325934;
			PSM->VM_CH2 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
			PSM->CM_CH2 = (1.2) / 8388607 / 0.5; //0.5mR

			break;

		case 2: //PSM in motor box
			PSM->VDCOS_CH1 = 0;
			PSM->CDCOS_CH1 = 0;
			PSM->VM_CH1 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
			PSM->CM_CH1 = (1.2) / 8388607 / 0.5; //0.5mR;

			break;

		case 3: //PSM in PPT box
			PSM->VDCOS_CH1 = 0;
			PSM->CDCOS_CH1 = 0;
			PSM->VM_CH1 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
			PSM->CM_CH1 = (1.2) / 8388607 / 2; //2mR;

			PSM->VDCOS_CH2 = 0;
			PSM->CDCOS_CH2 = 0;
			PSM->VM_CH2 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
			PSM->CM_CH2 = (1.2) / 8388607 / 0.5; //0.5mR;

			PSM->VDCOS_CH3 = 0;
			PSM->CDCOS_CH3 = 0;
			PSM->VM_CH3 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
			PSM->CM_CH3 = (1.2) / 8388607 / 2; //2mR;

			PSM->VDCOS_CH4 = 0;
			PSM->CDCOS_CH4 = 0;
			PSM->VM_CH4 = (1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)); //2.2kR;
			PSM->CM_CH4 = (1.2) / 8388607 / 2; //2mR;

			break;
	}
}

// arrayToDouble()
// Converts double data stored in an array back to double and returns the double
// Don't use this function here. Use it on the board that shall receive the voltage and current in array form from UART
/* @Param
 * 		aryPtr: pointer to the memory location of an element in an array
 * 		size: the number of consecutive array elements used to store data for the double value
 *
 */


//writeOnePSM()
//helper function for writing to ONE ade7912 with SPI
//PARAMETERS:
//spiInterface is the SPI pins that are used to communicate between the stm32 and the PSM
//uartInterface is the UART pins of the serial monitor that will output messages for debugging and information
//address = address of register in psm channel that you wish to write to
//data = 1 byte of data that you wish to write to the register at the specified address
//channelNumber = specifies which PSM channel you wish to write to
void writeOnePSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t address, uint8_t data, uint8_t channelNumber){
	//variables for error messages
	char errorMessage[64];
	uint8_t errorMessageLength;

	//16-bit write instruction to be sent to ade7912 chip in psm channel
	uint8_t instruction[2] = {address<<3, data}; //leftshift address bits to five most significant bits of instruction
	uint8_t dummyBuffer;
	//set specified chip select pin to 0 to start SPI communication
	switch(channelNumber){
		case 1:
			HAL_GPIO_WritePin(PSM->CSPort0, PSM->CSPin0, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(PSM->CSPort1, PSM->CSPin1, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(PSM->CSPort2, PSM->CSPin2, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(PSM->CSPort3, PSM->CSPin3, GPIO_PIN_RESET);
			break;
		default:
			//transmit error message "Invalid PSM channel number!"
			errorMessageLength = (uint8_t)sprintf(errorMessage, "(writeOnePSM) ERROR INVALID PSM CHANNEL NUMBER: %u\r\n", channelNumber);
			HAL_UART_Transmit(uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
			break;
	}

	if(HAL_SPI_TransmitReceive(spiInterface, instruction, &dummyBuffer, 2, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK){
		//successful transmission
	} else{
		//data could not be written! transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "(writeOnePSM) ERROR SENDING TO ADDRESS 0x%X WITH DATA 0x%X\r\n", address, data);
		HAL_UART_Transmit(uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}

	//set all chip select pins to 1 to disable further SPI transmission
	HAL_GPIO_WritePin(PSM->CSPort0, PSM->CSPin0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort1, PSM->CSPin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort2, PSM->CSPin2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort3, PSM->CSPin3, GPIO_PIN_SET);
}

//writeMultiplePSM()
//helper function for writing to multiple ade7912 with SPI
//PARAMETERS:
//spiInterface is the SPI pins that are used to communicate between the stm32 and the PSM
//uartInterface is the UART pins of the serial monitor that will output messages for debugging and information
//address = address of register in psm channel that you wish to write to
//data = 1 byte of data that you wish to write to the register at the specified address
//set EN_cx = 1 if you want to ENable configuration of PSM channel x, set EN_cx = 0 if you don't want to cNable configuration of PSM Channel x
//ex: EN_c2 = 1 means that configuration for PSM channel 2 is ENabled
void writeMultiplePSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t address, uint8_t data,
				uint8_t EN_c1, uint8_t EN_c2, uint8_t EN_c3, uint8_t EN_c4){
	//16-bit write instruction to be sent to ade7912 chip in psm channel
	uint8_t instruction[2] = {address<<3, data}; //leftshift address bits to five most significant bits of instruction
	uint8_t dummyBuffer;

	//set certain chip select pins to 0 to enable writing to specified PSM channels
	if(EN_c1){
		HAL_GPIO_WritePin(PSM->CSPort0, PSM->CSPin0, GPIO_PIN_RESET);
	}
	if(EN_c2){
		HAL_GPIO_WritePin(PSM->CSPort1, PSM->CSPin1, GPIO_PIN_RESET);
	}
	if(EN_c3){
		HAL_GPIO_WritePin(PSM->CSPort2, PSM->CSPin2, GPIO_PIN_RESET);
	}
	if(EN_c4){
		HAL_GPIO_WritePin(PSM->CSPort3, PSM->CSPin3, GPIO_PIN_RESET);
	}

	if(HAL_SPI_TransmitReceive(spiInterface, instruction, &dummyBuffer, 2, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK){
		//successful transmission
	} else{
		//data could not be written! transmit some error message to the computer
		char errorMessage[64];
		uint8_t errorMessageLength = (uint8_t)sprintf(errorMessage, "(writeMultPSM) ERROR SENDING TO ADDRESS 0x%X WITH DATA 0x%X\r\n", address, data);
		HAL_UART_Transmit(uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}

	//set all chip select pins to 1 to disable further SPI transmission
	HAL_GPIO_WritePin(PSM->CSPort0, PSM->CSPin0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort1, PSM->CSPin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort2, PSM->CSPin2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort3, PSM->CSPin3, GPIO_PIN_SET);
}

//readFromPSM()
//helper function for reading from ade7912 registers using SPI
//read from only ONE register at a time
//PARAMETERS:
//spiInterface is the SPI pins that are used to communicate between the stm32 and the PSM
//uartInterface is the UART pins of the serial monitor that will output messages for debugging and information
//address = address of ade7912 register that you want to read from
//buffer = pointer to buffer that will store received data
//numBytes = number of bytes you want to read from the register (because ade7912 registers are of varying sizes)
//channelNumber = number specifying which PSM channel you want to read from. For example, channelNumber = 2 means read from PSM channel 2
//MAKE SURE ENOUGH SPACE IN BUFFER TO ACCOMMODATE DATA, OTHERWISE SEGFAULT
void readFromPSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t address, uint8_t* buffer, uint16_t numBytes, uint8_t channelNumber){
	//variables for error messages
	char errorMessage[64];
	uint8_t errorMessageLength;

	//8-bit read instruction to be sent to ade7912 chip in psm channel
	//leftshift address bits to five most significant bits of instruction
	//Bit #2 needs to be set to 1 for read operation, thus must add 0b100 = 4 to instruction
	uint8_t instruction = (address<<3) + 0x04;
	//instruction = 0x4C;

	//set specified chip select pin to 0 to start SPI communication
	switch(channelNumber){
		case 1:
			HAL_GPIO_WritePin(PSM->CSPort0, PSM->CSPin0, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(PSM->CSPort1, PSM->CSPin1, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(PSM->CSPort2, PSM->CSPin2, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(PSM->CSPort3, PSM->CSPin3, GPIO_PIN_RESET);
			break;
		default:
			//transmit error message "Invalid PSM channel number!"
			errorMessageLength = (uint8_t)sprintf(errorMessage, "(readPSM) ERROR INVALID PSM CHANNEL NUMBER: %u\r\n", channelNumber);
			HAL_UART_Transmit(uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
			break;
	}

	//send read instruction to ade7912 in specified PSM channel
	if(HAL_SPI_TransmitReceive(spiInterface, &instruction, buffer, 1, MAX_SPI_TRANSMIT_TIMEOUT) == HAL_OK){ //Used HAL_SPI_Transmit alone and _Receive inside function second
		//successful transmission, so nothing to do
	} else{
		//instruction not sent!
		//transmit some error message to the computer
		errorMessageLength = (uint8_t)sprintf(errorMessage, "ERROR SENDING READ COMMAND TO ADDRESS 0x%X OF CHANNEL %u\r\n", address, channelNumber);
		HAL_UART_Transmit(uartInterface, (uint8_t*)errorMessage, (uint16_t)errorMessageLength, MAX_UART_TRANSMIT_TIMEOUT);
	}

	//set all chip select pins to 1 to disable further SPI transmission
	HAL_GPIO_WritePin(PSM->CSPort0, PSM->CSPin0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort1, PSM->CSPin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort2, PSM->CSPin2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->CSPort3, PSM->CSPin3, GPIO_PIN_SET);
}

//configPSM()
//configures PSM for operation
//PARAMETERS:
//spiInterface is the SPI pins that are used to communicate between the stm32 and the PSM
//uartInterface is the UART pins of the serial monitor that will output messages for debugging and information
//channels is a string containing the numbers of the channels you want to configure, ex: channels = "134" means configure PSM channels 1,3, and 4
void configPSM(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, char* channels, uint8_t master){
	//enable LVDS by outputting logic high at pin PB13
	// HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	uint8_t configCommand = CONFIG_COMMAND; //byte to be written to CONFIG register
    uint8_t BW, SWRST, ADC_FREQ, PWRDWN_EN, CLKOUT_EN; //control bits in configCommand

    //EN_cx = 1 means PSM channel x will be configured, EN_cx = 0 means PSM channel x will be ignored
    //for example EN_c2 = 1 means PSM channel 2 will be configured
    uint8_t EN_c1 = 0, EN_c2 = 0, EN_c3 = 0, EN_c4 = 0;

	//parse channels string to see which channels to write to by updating values of EN_cx
	for(uint8_t i = 0; channels[i] != '\0'; i++){
		switch(channels[i]){
			case '1':
				EN_c1 = 1;
				break;
			case '2':
				EN_c2 = 1;
				break;
			case '3':
				EN_c3 = 1;
				break;
			case '4':
				EN_c4 = 1;
				break;
		}
	}

    //channelStatus array stores bit 0 of STATUS0 register of each channel.
    //WARNING: indexing can be confusing. ex: channelStatus[1] corresponds to STATUS0[0] of PSM channel 2, channelStatus[3] corresponds to PSM channel 4, etc
	uint8_t channelStatus[4] = {1,1,1,1};
	uint8_t buffer;

	//datasheet specifies to initialize master ADE7912 before slave ADE7912, thus initializing PSM channel 2 before other PSM channels
    //wait for master ADE7912 (PSM channel 2) to be ready to accept commands
	//check if ade7912 of channel 2 is ready by reading bit 0 of STATUS0 register and seeing if it equals 0
    do{
		readFromPSM(PSM, spiInterface, uartInterface, STATUS0, &buffer, 1, master);
		channelStatus[master - 1] = buffer & 1; //buffer & 1 = STATUS0[0]
	}while(channelStatus[master - 1] && 0x01);

	if(EN_c2){
		if (USE_WRITE_PROTECTION){
			//disable any write-protection of PSM 2 config registers by writing 0x9C to its Lock register
			writeMultiplePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, UNLOCK_COMMAND, 0, EN_c2, 0, 0);
		}

	    //initialize CONFIG register of master ADE7912 (PSM channel 2)
		//for ONLY the channel #2 PSM that provides CLKOUT for other PSMs, must set CLKOUT_EN = 1 (this is OK because, on the single-channel PSM, the channel is #1)
		CLKOUT_EN = 1;
		if (PWR_DWN_ENABLE){	PWRDWN_EN = 1; } else {	PWRDWN_EN = 0;	}
	    configCommand |= CLKOUT_EN; //Set CLKOUT_EN (if single channel on PSM, CLKOUT_EN = 0. Else, CLKOUT_EN = 1)
	    configCommand |= (PWRDWN_EN << 2); //Set/reset PWRDWN_EN bit
		writeMultiplePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, 0, EN_c2, 0, 0);

		//initialize EMI_CTRL register for master ADE7912 (PSM channel 2)
		//as specified by wiki, write 0x55 for PSM 2
		writeMultiplePSM(PSM, spiInterface, uartInterface, EMI_CTRL, 0x55, 0, EN_c2, 0, 0);
	}

	//wait for slave PSM channels to be ready to accept commands
	//check if ade7912 is ready to accept commands by reading bit 0 of STATUS0 register and seeing if it equals 0
	do{
		if(EN_c1){
			readFromPSM(PSM, spiInterface, uartInterface, STATUS0, &buffer, 1, 1);
			channelStatus[0] = buffer & 1; //buffer & 1 = STATUS0[0]
		}
		if(EN_c3){
			readFromPSM(PSM, spiInterface, uartInterface, STATUS0, &buffer, 1, 3);
			channelStatus[2] = buffer & 1;
		}
		if(EN_c4){
			readFromPSM(PSM, spiInterface, uartInterface, STATUS0, &buffer, 1, 4);
			channelStatus[3] = buffer & 1;
		}
	//check if each specified channel has been configured yet
	}while((channelStatus[0] && EN_c1) || (channelStatus[2] && EN_c3) || (channelStatus[3] && EN_c4));

	if(EN_c1 || EN_c3 || EN_c4){
		if (USE_WRITE_PROTECTION){
			//disable any write-protection of slave PSM config registers by writing 0x9C to their Lock registers
			writeMultiplePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, UNLOCK_COMMAND, EN_c1, 0, EN_c3, EN_c4);
		}

		//initialize CONFIG registers of slave PSMs (channels 1,3,4)
		//write to PSM channels that receive the clkoutPSM channel clock, i.e. PSM channels 1,3,4
		if (PWR_DWN_ENABLE){	PWRDWN_EN = 1; } else {	PWRDWN_EN = 0;	}
	    configCommand |= (PWRDWN_EN << 2); //Set/reset PWRDWN_EN bit
		writeMultiplePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, EN_c1, 0, EN_c3, EN_c4);

		//initialize EMI_CTRL registers of slave ade7912 (psm channels 1,3,4)
		//as specified by wiki, write 0xAA for PSMs 1 and 3, 0x55 for PSM 4
		if(EN_c1 || EN_c3){
			writeMultiplePSM(PSM, spiInterface, uartInterface, EMI_CTRL, 0xAA, EN_c1, 0, EN_c3, 0);
		}
		if(EN_c4){
			writeMultiplePSM(PSM, spiInterface, uartInterface, EMI_CTRL, 0x55, 0, 0, 0, EN_c4);
		}
	}

	//initialize SYNC_SNAP register to 0x01 to synchronize all PSM channels
	writeMultiplePSM(PSM, spiInterface, uartInterface, SYNC_SNAP, 0x01, EN_c1, EN_c2, EN_c3, EN_c4);

	//TODO check for any transmission errors to several configuration registers using their CRC value, stored in CTRL_CRC register
	//actually I currently have no idea how to do this. Datasheet doesn't tell you what the a-bits are, how am I supposed to calculate and compare CRC without knowing that??
	//read from CTRL_CRC register (address: 0x5) and check configCRC value for any indication of error
//	uint16_t configCRC;
//	if(EN_c1){
//		readFromPSM(psmPorts, spiInterface, uartInterface, CTRL_CRC, (uint8_t*)&configCRC, 2, 1);
//	}
//	if(EN_c2){
//		readFromPSM(psmPorts, spiInterface, uartInterface, CTRL_CRC, (uint8_t*)&configCRC, 2, 2);
//	}
//	if(EN_c3){
//		readFromPSM(psmPorts, spiInterface, uartInterface, CTRL_CRC, (uint8_t*)&configCRC, 2, 3);
//	}
//	if(EN_c4){
//		readFromPSM(psmPorts, spiInterface, uartInterface, CTRL_CRC, (uint8_t*)&configCRC, 2, 4);
//	}

	if (USE_WRITE_PROTECTION){
		//write-protect the config registers of slave PSM channels by writing 0xCA to their Lock registers
		if(EN_c1 || EN_c3 || EN_c4){
			writeMultiplePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, LOCK_COMMAND, EN_c1, 0, EN_c3, EN_c4);
		}
	}

	//disable CLKOUT of PSM channel 2 by setting CLKOUT_EN = 0 to conserve power
//	if(EN_c2){
//		//since value of configCommand is already set so that CLKOUT_EN = 0, just write it to PSM channel 2
//		writeMultiplePSM(psmPorts, spiInterface, uartInterface, CONFIG, configCommand, 0, EN_c2, 0, 0);
//
//		//write-protect the config registers of the master PSM channel by writing 0xCA to its Lock register
//		writeMultiplePSM(psmPorts, spiInterface, uartInterface, LOCK_ADDRESS, LOCK_COMMAND, 0, EN_c2, 0, 0);
//	}

	//disable LVDS by outputting logic low to pin PB13
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);
}

//PSMRead()
//read voltage and current measurements from a PSM
//fills dataOut[] and returns voltage in dataOut[0], current in dataOut[1]. Values are stored as doubles
//if dataOutLen is not set to 2, dataOut[] will not be filled
//PARAMETERS:
//spiInterface is the SPI pins that are used to communicate between the stm32 and the PSM
//uartInterface is the UART pins of the serial monitor that will output messages for debugging and information
//masterPSM is the PSM channel number of the master ade7912
//channelNumber is the PSM channel that will be read from
//CLKOUT indicates whether the master PSM's clock should be enabled for the slave PSMs. set to 1 to enable, 0 to disable

//!!! If there is only ONE PSM channel in total, set masterPSM = 0 !!!
//ex: masterPSM = 2 means that PSM channel 2 provides the clock.
void PSMRead(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t CLKOUT, uint8_t masterPSM, uint8_t channelNumber, double dataOut[], uint8_t dataOutLen){
	//enable LVDS by outputting logic high at pin PB13

	// HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	uint8_t configCommand = 0; //byte to be written to CONFIG register
	uint8_t dataIn[6] = {0};//data received from ade7912
//	uint8_t dataOut[16] = {0}; //data to be sent through UART
//	uint8_t dataOut[17] = {0}; // New dataOut for BlueSky Protocol
//	uint8_t dataOutLen = sizeof(dataOut) / sizeof(dataOut[0]); //dataOut length in bytes
	uint32_t IWV_val, V1WV_val; //for storing data from IWV and V1WV registers of ade7912 respectively
	double voltage = 0, current = 0;
	double voltageOffset = 0, currentOffset = 0, voltageMultiplier = 1, currentMultiplier = 1;

	//assign offset and multiplier values depending on channelNumber
	switch(channelNumber){
		case 1:
			voltageOffset = PSM->VDCOS_CH1;
			currentOffset = PSM->CDCOS_CH1;
			voltageMultiplier = PSM->VM_CH1;
			currentMultiplier = PSM->CM_CH1;
			break;
		case 2:
			voltageOffset = PSM->VDCOS_CH2;
			currentOffset = PSM->CDCOS_CH2;
			voltageMultiplier = PSM->VM_CH2;
			currentMultiplier = PSM->CM_CH2;
			break;
		case 3:
			voltageOffset = PSM->VDCOS_CH3;
			currentOffset = PSM->CDCOS_CH3;
			voltageMultiplier = PSM->VM_CH3;
			currentMultiplier = PSM->CM_CH3;
			break;
		case 4:
			voltageOffset = PSM->VDCOS_CH4;
			currentOffset = PSM->CDCOS_CH4;
			voltageMultiplier = PSM->VM_CH4;
			currentMultiplier = PSM->CM_CH4;
			break;
		default:
			//invalid PSM number TODO add error message
			break;
	}

	if (USE_WRITE_PROTECTION){
		//disable write protection
		writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, UNLOCK_COMMAND, channelNumber);
	}

	//enable master ade7912 clock output by setting CLKOUT_EN = 1
	if(masterPSM){
		//disable write protection
		if (USE_WRITE_PROTECTION){
			writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, UNLOCK_COMMAND, masterPSM);
		}

		readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, masterPSM);

		configCommand |= 1; //CLKOUT_EN = 1
	    configCommand &= (~(1<<2)); //PWRDWN_EN = 0
	    configCommand &= (~(1<<5)); //SWRST = 0

	    writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, masterPSM);
	}

	//wake up slave ade7912
    readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, channelNumber);
    configCommand &= (~(1<<2)); //PWRDWN_EN = 0
    configCommand &= (~(1<<5)); //SWRST = 0
    writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, channelNumber);

    //read from selected ade7912
    for(uint8_t i = 0; i < NUM_AVG; i++){
        //wait for DREADY_n signal to go low before reading (pin PK2)
        while(HAL_GPIO_ReadPin(PSM->DreadyPort, PSM->DreadyPin) == GPIO_PIN_SET){
        } //TODO add something that will get us out of this loop if PK2 never reaches low
        //initiate burst-read mode by sending read instruction with address IWV, read 6 bytes

        readFromPSM(PSM, spiInterface, uartInterface, IWV, dataIn, 3, channelNumber);
        IWV_val = (dataIn[0] << 16) + (dataIn[1] << 8) + dataIn[2];

        readFromPSM(PSM, spiInterface, uartInterface, V1WV, dataIn, 3, channelNumber);
        V1WV_val = (dataIn[0] << 16) + (dataIn[1] << 8) + dataIn[3];

        current += (IWV_val - currentOffset)*currentMultiplier;
        voltage += (V1WV_val - voltageOffset)*voltageMultiplier;
//        current += (IWV_val - currentOffset)*((1.2) / 8388607 / 0.5);
//        voltage += (V1WV_val - voltageOffset)*((1.2) / 8388607 * (1 + 664*(1/2.2 + 1/480)));
    }
    //get averages of current and voltage measurements
    voltage /= NUM_AVG;
    current /= NUM_AVG;

    /* ========== not doing this here ======================= */
    //convert voltage and current from doubles to uint8_t array that can be sent through uart
    //voltage will make up 8 most significant bytes, current will make up 8 least significant bytes of array
    //TODO CHECK ENDIANNESS !!!

    uint8_t dataOutIndex = 0;
    uint8_t* ptr = (uint8_t*)&voltage;

    for(; dataOutIndex < sizeof(voltage); dataOutIndex++){
    	dataOut[dataOutIndex] = *ptr;
    	ptr++;
    }
    ptr = (uint8_t*)&current;
    for(; dataOutIndex < dataOutLen; dataOutIndex++){
    	dataOut[dataOutIndex] = *ptr;
    	ptr++;
    }

    if (dataOutLen == 2) {
    	dataOut[0] = voltage;
    	dataOut[1] = current;
    }

    //power down ade7912
    if (PWR_DWN_ENABLE){
        readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, channelNumber);
        configCommand |= (1<<2); //PWRDWN_EN = 1
        writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, channelNumber);

    	//disable master ade7912 clock output by setting CLKOUT_EN = 0
        //power down master ade7912 as well by setting PWRDWN_EN = 1
    	readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, masterPSM);
        configCommand &= (~1); //CLKOUT_EN = 0
        configCommand |= (1<<2); //PWRDWN_EN = 1
        writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, masterPSM);
    }

	//re-enable write-protection
    if (USE_WRITE_PROTECTION){
    	writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, LOCK_COMMAND, channelNumber);

    	if(masterPSM){
    		writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, LOCK_COMMAND, masterPSM);
    	}
    }

	//disable LVDS by outputting logic low to pin PB13
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);
}

//PSMPowerDown()
//
void PSMPowerDown(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, char* channels){

}

//PSMReadTemperature()
//masterPSM is the channel number of the master ade7912
//read temperature from a specified PSM channel
//casts temperature value from int16_t to two uint8_t for UART transmission, MAKE SURE TO RECAST THE RECEIVED TEMPERATURE VALUE BACK TO SIGNED 16-BIT INTEGER BEFORE USE
void PSMReadTemperature(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, uint8_t masterPSM){
	//enable LVDS by outputting logic high at pin PB13
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	//variables
	uint8_t configCommand = 0; //byte to be sent to PSM channel
	uint8_t TEMPOS_val = 0; //value from TEMPOS register
	uint8_t dataIn[3] = {0}; //data received from ade7912
	int16_t dataOut = 0; //temperature data sent through UART
	uint32_t V2WV_val = 0; //value from V2WV register
	double gain = 0; //gain value depends on BW (CONFIG[7]) values as outlined in ade7912 datasheet
	double temperature = 0;

	if (USE_WRITE_PROTECTION){
		//disable write protection
		writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, UNLOCK_COMMAND, masterPSM);
	}

    readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, masterPSM);
    //assign gain value depending on BW (CONFIG[7])
    if((configCommand>>7) & 1){ //if BW = 1
    	gain = 8.21015e-5;
    } else{ //BW = 0
    	gain = 8.72101e-5;
    }
	//wakeup master ADE7912 by setting PWRDWN_EN = 0
    configCommand &= (~(1<<2)); //PWRDWN_EN = 0
    writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, masterPSM);

    //get TEMPOS value
    readFromPSM(PSM, spiInterface, uartInterface, TEMPOS, &TEMPOS_val, 1, masterPSM);

    for(uint8_t i = 0; i<10; i++){
        //wait for DREADY_n signal to go low before reading (pin PK2)
        while(HAL_GPIO_ReadPin(PSM->DreadyPort, PSM->DreadyPin) == GPIO_PIN_SET){
        } //TODO add something that will get us out of this loop if PK2 never reaches low
        //get V2WV value
        readFromPSM(PSM, spiInterface, uartInterface, V2WV, dataIn, 3, masterPSM);
        V2WV_val = (dataIn[0]<<16)+(dataIn[1]<<8)+dataIn[2];
        temperature += gain*V2WV_val + 8.72101e-5*TEMPOS_val*2048 - 306.47; //formula from ade7912 datasheet
    }
    //take average of 10 temperature readings from master ade7912
    temperature /= 10;

    //since this value is not for precise measurement, will round the temperature to int16_t and send this instead to facilitate uart
    dataOut = (int16_t)(temperature < 0 ? (temperature - 0.5) : (temperature + 0.5));
    //DATA IS SENT THROUGH UART AS TWO UNSIGNED 8 BIT INT, MAKE SURE TO RECAST VALUE BACK TO ONE SIGNED 16BIT INT AT RECEIVER BEFORE USE
    //TODO CHECK ENDIANNNESS
    //HAL_UART_Transmit(uartInterface, (uint8_t *)&dataOut, 2, MAX_UART_TRANSMIT_TIMEOUT);
    HAL_UART_Transmit(uartInterface, (uint8_t *)&dataOut, sizeof(dataOut), MAX_UART_TRANSMIT_TIMEOUT);

    //powerdown master ade7912 by setting PWRDWN_EN = 1
    readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, masterPSM);
    configCommand |= (1<<2); //PWRDWN_EN = 1
    writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, masterPSM);

    if (USE_WRITE_PROTECTION){
    	//re-enable write protection
    	writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, LOCK_COMMAND, masterPSM);
    }

	//disable LVDS by outputting logic low to pin PB13
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);
}

//PSMCalib()
//Call this function when calibrating ONE specific PSM channel
//estimates DC offset of IWV and V1WV channels
//PARAMETERS:
//voltageToInputRatio = V_in/V_out (voltage divider ratio)
//shuntResistance is entered in Ohms.
//channelNumber is the number of the specific channel that is to be calibrated

//SET masterPSM = 0 IF THERE IS ONLY ONE PSM CHANNEL IN TOTAL!
void PSMCalib(struct PSM_Peripheral* PSM, SPI_HandleTypeDef* spiInterface, UART_HandleTypeDef* uartInterface, double voltageToInputRatio,
		double shuntResistance, uint8_t masterPSM, uint8_t channelNumber){
	//enable LVDS by outputting logic high at pin PB13
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_SET);

	uint8_t configCommand = 0; //byte to be written to CONFIG register
	uint8_t dataIn[6] = {0};//data received from ade7912
	uint8_t dataOut[256] = {0}; //data to be sent through UART
	uint16_t dataOutLen = 256; //length of dataOut, in bytes
	double voltageOffset_TH = 0, currentOffset_TH = 0; //THEORETICAL offsets
	double voltageMultiplier_TH = 1, currentMultiplier_TH = 1; //THEORETICAL multipliers

	//calculate theoretical multipliers
	voltageMultiplier_TH = 0.5/(5320000*voltageToInputRatio);
	currentMultiplier_TH = 0.03125/(5320000*shuntResistance);

	//disable write protection if needed
	if (USE_WRITE_PROTECTION){	writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, UNLOCK_COMMAND, channelNumber); }

	//enable master ade7912 clock output by setting CLKOUT_EN = 1
	if(masterPSM){
		//disable write protection
		if (USE_WRITE_PROTECTION){	writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, UNLOCK_COMMAND, masterPSM); }
		readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, masterPSM);
		configCommand |= 1; //CLKOUT_EN = 1
	    writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, masterPSM);
	}

	//wake up slave ade7912
    readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, channelNumber);
    configCommand &= (~(1<<2)); //PWRDWN_EN = 0
    writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, channelNumber);

    //read from selected ade7912
    for(uint8_t i = 0; i<50; i++){
        //wait for DREADY_n signal to go low before reading (pin PK2)
        while(HAL_GPIO_ReadPin(PSM->DreadyPort, PSM->DreadyPin) == GPIO_PIN_SET){
        } //TODO add something that will get us out of this loop if PK2 never reaches low
        //initiate burst-read mode by sending read instruction with address IWV, read 6 bytes
        readFromPSM(PSM, spiInterface, uartInterface, IWV, dataIn, 6, channelNumber);
        currentOffset_TH += (dataIn[0]<<16) + (dataIn[1]<<8) + dataIn[2]; //IWV value
        voltageOffset_TH += (dataIn[3]<<16) + (dataIn[4]<<8) + dataIn[5]; //V1WV value
    }
    //get averages of 50 IWV and V1WV measurements to get theoretical offsets
    voltageOffset_TH /= 50;
    currentOffset_TH /= 50;

    //output offsets and multipliers through uart to serial monitor
    dataOutLen = (uint16_t)sprintf((char*)dataOut,
    		"CALIBRATE CHANNEL %d, V_in/V_out = %lf, shuntResistance = %lf Ohm\r\n"
    		"THEORETICAL voltageMultiplier = %lf\r\n"
    		"THEORETICAL currentMultiplier = %lf\r\n"
    		"THEORETICAL voltageOffset = %lf\r\n"
    		"THEORETICAL currentOffset = %lf\r\n",
			channelNumber,
			voltageToInputRatio,
			shuntResistance,
			voltageMultiplier_TH,
			currentMultiplier_TH,
			voltageOffset_TH,
			currentOffset_TH);
    HAL_UART_Transmit(uartInterface, dataOut, dataOutLen, MAX_UART_TRANSMIT_TIMEOUT);

    //power down slave ade7912
    if (PWR_DWN_ENABLE){
        readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, channelNumber);
        configCommand |= (1<<2); //PWRDWN_EN = 1
        writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, channelNumber);

    	//disable master ade7912 clock output by setting CLKOUT_EN = 0
        //power down master ade7912 as well by setting PWRDWN_EN = 1
    	readFromPSM(PSM, spiInterface, uartInterface, CONFIG, &configCommand, 1, masterPSM);
        configCommand &= (~1); //CLKOUT_EN = 0
        configCommand |= (1<<2); //PWRDWN_EN = 1
        writeOnePSM(PSM, spiInterface, uartInterface, CONFIG, configCommand, masterPSM);
    }

    if (USE_WRITE_PROTECTION){
    	//re-enable write-protection
    	writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, LOCK_COMMAND, channelNumber);

    	if (masterPSM){
    		writeOnePSM(PSM, spiInterface, uartInterface, LOCK_ADDRESS, LOCK_COMMAND, masterPSM);
    	}
    }

	//disable LVDS by outputting logic low to pin PB13
	HAL_GPIO_WritePin(PSM->LVDSPort, PSM->LVDSPin, GPIO_PIN_RESET);
}
