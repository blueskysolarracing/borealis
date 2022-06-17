#include "stm32l4xx_hal_spi.h"


#ifndef JEFFSSHITCODEBMS
#define JEFFSSHITCODEBMS


/*======plz read the stuff below and it will save time
 * there are 3 low level functions:
 * 	1.commandToArray 2.generatePECbits
 * the function  LTC6810CommandGenerate(int command, uint8_t dataToSend[]) is the
 * function that u want to use in main.c, other functions are all supporting functions.
 *
 * the LTC6810checkAllVoltage(int voltageArray[]) is a function
 *
 */

void generatePECbits(int CMD0[],int CMD1[], int PCE0[], int PCE1[]){

	//refer to P55 of LTC
	int IN0,IN3,IN4,IN7,IN8,IN10,IN14;
	IN0=IN3=IN4=IN7=IN8=IN10=IN14=0; //initialize all polynomial stuff to 0

	int PEC[15];
	int i=0;
	while(i<15){
		PEC[i] = 0;
		i++;
	}
	PEC[4] = 1;
	//above initialize the 15 bit PEC. in last cycle shift up add 0 at LSB

	//Count Din from most significant bit
	int DIN;//
	for(i=15;i >= 0; i--){//do the shifting and stuff

		if(i>=8){//grab DIN from CMD0
			DIN = CMD0[i-8];
		}
		else{
			DIN = CMD1[i];
		}
		//now DIN is assigned, do step 2 which is shifting
		IN0 = DIN ^ PEC[14];
		IN3 = IN0 ^ PEC [2];
		IN4 = IN0 ^ PEC [3];
		IN7 = IN0 ^ PEC [6];
		IN8 = IN0 ^ PEC [7];
		IN10 = IN0 ^ PEC [9];
		IN14 = IN0 ^ PEC [13];

		//step 3, shift
		PEC [14] = IN14;
		PEC [13] = PEC [12];
		PEC [12] = PEC [11];
		PEC [11] = PEC [10];
		PEC [10] = IN10;
		PEC [9] = PEC [8];
		PEC [8] = IN8;
		PEC [7] = IN7;
		PEC [6] = PEC [5];
		PEC [5] = PEC [4];
		PEC [4] = IN4;
		PEC [3] = IN3;
		PEC [2] = PEC [1];
		PEC [1] = PEC [0];
		PEC [0] = IN0;

		//printf(" %d \n",i);
		//printf("Din: %d 0:%d 3:%d 4:%d 7:%d 8:%d 10:%d 14:%d \n",DIN,IN0,IN3,IN4,IN7,IN8,IN10,IN14 );

	}//end of main shifting for loop
	int j;
	for(j=14;j>=0;j--){
	    //printf("%d",PEC[j]);   ///test
	    if(j>=7){
	       PCE0[j-7] = PEC[j];
	    }
	    else{
	        PCE1[j+1] = PEC[j];
	    }
	}
	//now add a 0 at the end and stuff it into 2 bytes
	PCE1[0]=0;

}//====== end of generatePCEbits function =========

void generatePECbits6Byte(int data6Byte[], int PCE0[], int PCE1[]){
    //Note: data6Byte[74] is 1st bit

	//refer to P55 of LTC
	int IN0,IN3,IN4,IN7,IN8,IN10,IN14;
	IN0=IN3=IN4=IN7=IN8=IN10=IN14=0; //initialize all polynomial stuff to 0

	int PEC[15];
	int i=0;
	while(i<15){
		PEC[i] = 0;
		i++;
	}
	PEC[4] = 1;
	//above initialize the 15 bit PEC. in last cycle shift up add 0 at LSB

	//Count Din from most significant bit
	int DIN;//
	for(i=47;i >= 0; i--){//do the shifting and stuff

		DIN = data6Byte[i];
		//now DIN is assigned, do step 2 which is shifting
		IN0 = DIN ^ PEC[14];
		IN3 = IN0 ^ PEC [2];
		IN4 = IN0 ^ PEC [3];
		IN7 = IN0 ^ PEC [6];
		IN8 = IN0 ^ PEC [7];
		IN10 = IN0 ^ PEC [9];
		IN14 = IN0 ^ PEC [13];

		//step 3, shift
		PEC [14] = IN14;
		PEC [13] = PEC [12];
		PEC [12] = PEC [11];
		PEC [11] = PEC [10];
		PEC [10] = IN10;
		PEC [9] = PEC [8];
		PEC [8] = IN8;
		PEC [7] = IN7;
		PEC [6] = PEC [5];
		PEC [5] = PEC [4];
		PEC [4] = IN4;
		PEC [3] = IN3;
		PEC [2] = PEC [1];
		PEC [1] = PEC [0];
		PEC [0] = IN0;

		//printf(" %d \n",i);
		//printf("Din: %d 0:%d 3:%d 4:%d 7:%d 8:%d 10:%d 14:%d \n",DIN,IN0,IN3,IN4,IN7,IN8,IN10,IN14 );

	}//end of main shifting for loop
	int j;
	for(j=14;j>=0;j--){
	    //printf("%d",PEC[j]);   ///test
	    if(j>=7){
	       PCE0[j-7] = PEC[j];
	    }
	    else{
	        PCE1[j+1] = PEC[j];
	    }
	}
	//now add a 0 at the end and stuff it into 2 bytes
	PCE1[0] = 0;

}//====== end of generatePECbits6Byte function=====


uint8_t arrayToByte(int arrayIn[]){ //8 bit array to a byte
	uint8_t outputByte = 0;
	int i=7;
	//note: [7] is MSB, [0] is LSB.
	while(i>=1){
		outputByte = outputByte + arrayIn[i];
		outputByte = outputByte * 2; //left shift one bit
		i--;
		//note: arrayIn[i] shall be either 1 or 0;
	}
	outputByte = outputByte + arrayIn[i];//put last bit in
	return outputByte;
}//======== end of arrayToByte function ========



void commandToArray(int command, int CMD0ref[], int CMD1ref[]){
	int tempBit;//stores the temporary bit value from command that goes into CMD0 or 1

	int i;
	for(i = 0; i<=15;i++){ //from LSB to MSB, fill CMD1 first
		tempBit = (command % 2 );
		if(i<=7){
			CMD1ref[i] = tempBit;
		}
		else{
			CMD0ref[i-8] = tempBit;
		}
		command = command >> 1; //right shift to remove the LSB
		// the ">>" is a binary rightshift command
	}

}//======== end of commandToArray function ========


void LTC6810CommandGenerate(int command, uint8_t dataToSend[]){ //dataToSend is array pass by reference
	int CMD0Array[8] = {0,0,0,0,0,0,0,0};
	int CMD1Array[8] = {0,0,0,0,0,0,0,0};//[7] is MSB, [0] is LSB
	//convert command and fill the two above array
	commandToArray(command, CMD0Array, CMD1Array);

	//now generate Error checking bits
	int PECbits0[8] = {0,0,0,0,0,0,0,0};
	int PECbits1[8] = {0,0,0,0,0,0,0,0};

	generatePECbits(CMD0Array, CMD1Array, PECbits0, PECbits1);

	//put them back into the dataToSend array, with size of 4

	dataToSend[0] = arrayToByte(CMD0Array);//1st byte message
	dataToSend[1] = arrayToByte(CMD1Array);//2nd byte message

	dataToSend[2] = arrayToByte(PECbits0);//1st byte error check
	dataToSend[3] = arrayToByte(PECbits1);//2nd byte error check

} // ======== end of LTC6810CommandGenerate function ==========

/*
void LTC6810checkAllVoltage(int voltageArray[]){//Array of 6 voltages
	//voltafeArray[1] is voltage of cell 0; [5] is voltage of cell 6
	//also this function will take a bit time cuz the ADC thing takes time

	int LTC6810Command;  //this variable carries the commands, see on page 60 of datasheet
	uint8_t dataToSend[4]; //converted from the binary command to data bytes
	uint8_t dataToReceive[8];

	LTC6810Command = 0b01011000000; //ADCV of cell 1;
	//MD = 01, 27kHz;  DCP=0, no discharge permit; Channel = 000: All channel;
	LTC6810CommandGenerate(LTC6810Command, dataToSend );//array passed by reference

	HAL_GPIO_WritePin (GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //slave select low, transmit begin
	HAL_SPI_Transmit(&hspi3, dataToSend,4,100);
	HAL_Delay(10);//ADD 10ms DELAY between Transmit & Receive
	HAL_SPI_Receive(&hspi3, dataToReceive,4,100);         //discard this Receive as it is problematic
	HAL_GPIO_WritePin (GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

	HAL_Delay(5);

	//after letting it read ADC, now collect those data

	LTC6810Command = 0b00000000100;
		//RDCVA read cell voltage register group A (cell 1-3)
	LTC6810CommandGenerate(LTC6810Command, dataToSend );
	HAL_GPIO_WritePin (GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //slave select low, transmit begin
	HAL_SPI_Transmit(&hspi3, dataToSend,4,100);
	HAL_Delay(2);
	HAL_SPI_Receive(&hspi3, dataToReceive,6,100);         //read 6 bytes, cell0,1,2
	HAL_GPIO_WritePin (GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //ss pin back high

	voltageArray[0] = (dataToReceive[1]*256) + dataToReceive[0];
	//lsb comes first, refer to page 64 Memory map
	voltageArray[1] = (dataToReceive[3]*256) + dataToReceive[2];
	voltageArray[2] = (dataToReceive[5]*256) + dataToReceive[4];

	LTC6810Command = 0b00000000110;
		//RDCVA read cell voltage register group B (cell 4-6)
	LTC6810CommandGenerate(LTC6810Command, dataToSend );
	HAL_GPIO_WritePin (GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //slave select low, transmit begin
	HAL_SPI_Transmit(&hspi3, dataToSend,4,100);
	HAL_Delay(2);
	HAL_SPI_Receive(&hspi3, dataToReceive,6,100);         //read 6 bytes, cell0,1,2
	HAL_GPIO_WritePin (GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //ss pin back high

	voltageArray[3] = (dataToReceive[1]*256) + dataToReceive[0];
	voltageArray[4] = (dataToReceive[3]*256) + dataToReceive[2];
	voltageArray[5] = (dataToReceive[5]*256) + dataToReceive[4];


}*/

int LTC6810initializeAndCheck(){
	//Call this function when starting up.
/*
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| reg   | Bit7    | Bit6    | Bit5    | Bit4    | Bit3    | Bit2    | Bit1    | Bit0   |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR0 | RSVD    | GPIO4   | GPIO3   | GPIO2   | GPIO1   | REFON   | DTEN    | ADCOPT |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR1 | VUV[7]  | VUV[6]  | VUV[5]  | VUV[4]  | VUV[3]  | VUV[2]  | VUV[1]  | VUV[0] |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR2 | VOV[3]  | VOV[2]  | VOV[1]  | VOV[0]  | VUV[11] | VUV[10] | VUV[9]  | VUV[8] |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR3 | VOV[11] | VOV[10] | VOV[9]  | VOV[8]  | VOV[7]  | VOV[6]  | VOV[5]  | VOV[4] |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR4 | DCC0    | MCAL    | DCC6    | DCC5    | DCC4    | DCC3    | DCC2    | DCC1   |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR5 | DCTO[3] | DCTO[2] | DCTO[1] | DCTO[0] | SCONV   | FDRF    | DIS_RED | DTMEN  |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+*/

//above is the table of configuration register bits. How
	int LTC6810Command;
	uint8_t dataToSend0[4]; //stores the command and PEC bit of WRCFG
	uint8_t dataToSend1[4]; //stores CFGR0 & CFGR1 stuff
	uint8_t dataToSend2[4]; //stores byte 2 & 3
	uint8_t dataToSend3[4]; //stores byte 4&5
	uint8_t dataToReceive[8];
//set GPIO bits to 1 so they aren`t being pulled down internally by the chip.
//set REFON to enable the 3V that goes to the chip
//DTEN to 0 to disable discharge timer
//ADCOPT bit to 0, use 422Hz as its stable
//above are byte0, byte 1 full of 0s as VUV currently not used.


return 0;//temp

}//====== End of LTC6810initializeAndCheck ======

int voltageDataConversion(uint8_t lowByte, uint8_t highByte){
	//based on LTC6810 datasheet, every bit is 100uV
	//full range of 16 bytes: -0.8192 to + 5.7344
	//Final voltage: (total - 8192) * 100uV
	int Voltage;
	Voltage = lowByte + highByte * 256;
	//Voltage = Voltage + 8192;
	return Voltage;
}

#endif
