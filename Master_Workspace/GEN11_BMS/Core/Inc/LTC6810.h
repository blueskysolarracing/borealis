#include "main.h"

#define DEBUG_EN 0 //When 1, print debug info to UART3

extern uint8_t dataToSend[16];
extern int messageInBinary; //write this in binary. This goes into [LTC6810CommandGenerate]
//extern uint8_t dataToReceive[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };//voltage data from LTC6810 via SPI

void readVolt(uint16_t* voltArray, SPI_HandleTypeDef *spi, UART_HandleTypeDef* uart);
void readTemp(uint16_t* tempArray, int DCC5, int DCC4, int DCC3, int DCC2, int DCC1, SPI_HandleTypeDef *spi, UART_HandleTypeDef* uart);
void generatePECbits(int CMD0[],int CMD1[], int PCE0[], int PCE1[]);
void generatePECbits6Byte(int data6Byte[], int PCE0[], int PCE1[]);
uint8_t arrayToByte(int arrayIn[]); //8 bit array to a byte
void commandToArray(int command, int CMD0ref[], int CMD1ref[]);
void LTC6810CommandGenerate(int command); //dataToSend is array pass by reference
int LTC6810Init(SPI_HandleTypeDef *spi, int GPIO4, int GPIO3, int GPIO2 ,int DCC5, int DCC4, int DCC3, int DCC2, int DCC1);
int voltageDataConversion(uint8_t lowByte, uint8_t highByte);
