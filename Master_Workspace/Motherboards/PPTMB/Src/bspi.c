/*
 * bspi.c
 *
 *  Created on: Sep. 26, 2019
 *      Author: Tom B
 */
#include "bspi.h"
#include "bgpio.h"

#define SPI_TASK_PRIORITY 3
#define SPI_TASK_STACK_SIZE 256
#define SPI_Q_SIZE 64
#define NUM_SPIS 3

// ########  ##     ##
// ##     ## ##     ##
// ##     ## ##     ##
// ########  ##     ##
// ##         ##   ##
// ##          ## ##
// ##           ###

static B_spiHandle_t* bspis[NUM_SPIS];
static SPI_HandleTypeDef* hspis[NUM_SPIS];

// ########  ######## ########
// ##     ## ##       ##     ##
// ##     ## ##       ##     ##
// ########  ######   ########
// ##        ##       ##
// ##        ##       ##
// ##        ##       ##


B_spiQMsg_t* readSpi(B_spiHandle_t *badc);
void doneReadSpi(B_spiQMsg_t *buf);
B_spiHandle_t* B_spiStart(SPI_HandleTypeDef *hspi, uint8_t numCS);
static void spiTask(void* pv);

// ######## ##     ## ##    ##  ######
// ##       ##     ## ###   ## ##    ##
// ##       ##     ## ####  ## ##
// ######   ##     ## ## ## ## ##
// ##       ##     ## ##  #### ##
// ##       ##     ## ##   ### ##    ##
// ##        #######  ##    ##  ######

B_spiHandle_t* B_spiStart(SPI_HandleTypeDef *hspi, uint8_t numCS){
	B_spiHandle_t *bspi;
	for(int i = 0; i < NUM_SPIS; i++){
		if(hspis[i] == NULL){
			hspis[i] = hspi;
			bspis[i] = pvPortMalloc(sizeof(B_spiHandle_t));
			bspi = bspis[i];
			break;
		}
	}
	bspi->hspi = hspi;
	bspi->spiQ = xQueueCreate(SPI_Q_SIZE, sizeof(B_spiQMsg_t *));
	bspi->numCS = numCS;
	bspi->spiSem = xSemaphoreCreateBinary();
	xTaskCreate(spiTask, "spiTask", SPI_TASK_STACK_SIZE, bspi, SPI_TASK_PRIORITY, &bspi->spiTask);
	return bspi;
}


static void spiTask(void* pv){
	B_spiHandle_t *bspi = pv;
	uint16_t *buf;
	buf= pvPortMaloc(16);
	for(;;){
		for(int i = 0; i < bspi->numCS; i++){
			//B_gpioSet(bspi->csPins[i]);
			HAL_SPI_TransmitReceive_DMA(bspi->hspi, buf, buf+8, 8);
		}
	}
}

//  ######     ###    ##       ##       ########     ###     ######  ##    ##
// ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
// ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
// ##       ##     ## ##       ##       ########  ##     ## ##       #####
// ##       ######### ##       ##       ##     ## ######### ##       ##  ##
// ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
//  ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
	for(int i = 0; i < NUM_SPIS; i++){
		if(hspis[i] == hspi){
			xSemaphoreGiveFromISR(bspis[i]->spiSem, NULL);
		}
	}
}
