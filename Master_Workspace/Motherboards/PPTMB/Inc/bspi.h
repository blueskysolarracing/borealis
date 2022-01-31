#include "main.h"
#include "cmsis_os.h"

typedef struct{
	uint16_t data;
}B_spiQMsg_t;

typedef struct{
	SPI_HandleTypeDef* hspi;
	QueueHandle_t 			spiQ;
	TaskHandle_t			spiTask;
	uint8_t					numCS;
	SemaphoreHandle_t 		spiSem;
}B_spiHandle_t;

B_spiQMsg_t* readSpi(B_spiHandle_t *badc);
void doneReadSpi(B_spiQMsg_t *buf);
B_spiHandle_t* B_spiStart(SPI_HandleTypeDef *hspi, uint8_t numCS);

