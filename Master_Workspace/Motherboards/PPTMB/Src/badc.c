#include "badc.h"

#define NUM_ADCS 3
#define ADC_TASK_PRIORITY 3
#define ADC_TASK_STACK_SIZE 256
#define ADC_BUFFER_LEN 128
#define ADC_Q_SIZE 64
// ########  ##     ##
// ##     ## ##     ##
// ##     ## ##     ##
// ########  ##     ##
// ##         ##   ##
// ##          ## ##
// ##           ###

static B_adcHandle_t* badcs[NUM_ADCS];
static ADC_HandleTypeDef* hadcs[NUM_ADCS];

// ########  ######## ########
// ##     ## ##       ##     ##
// ##     ## ##       ##     ##
// ########  ######   ########
// ##        ##       ##
// ##        ##       ##
// ##        ##       ##


B_adcHandle_t* B_adcStart(ADC_HandleTypeDef* hadc, uint8_t numChannels);
static void adcTask(void* pv);


// ######## ##     ## ##    ##  ######
// ##       ##     ## ###   ## ##    ##
// ##       ##     ## ####  ## ##
// ######   ##     ## ## ## ## ##
// ##       ##     ## ##  #### ##
// ##       ##     ## ##   ### ##    ##
// ##        #######  ##    ##  ######

B_adcHandle_t* B_adcStart(ADC_HandleTypeDef* hadc, uint8_t numChannels){
	B_adcHandle_t *badc;
    for(int i = 0; i < NUM_ADCS; i++){
        if(badcs[i] == NULL){
            badcs[i] = pvPortMalloc(sizeof(B_adcHandle_t));
            badc = badcs[i];
            hadcs[i] = hadc;
            badc->hadc = hadc;
            break;
        }
    }
	badc->adcQ = xQueueCreate(ADC_Q_SIZE, sizeof(uint16_t*));
	badc->numChannels = numChannels;
	badc->adcSem = xSemaphoreCreateBinary();
	xTaskCreate(adcTask, "adcTask", ADC_TASK_STACK_SIZE, badc, ADC_TASK_PRIORITY, &badc->adcTask);
	return badc;
}


uint16_t* readAdc(B_adcHandle_t *badc){
	uint16_t *buf;
	xQueueReceive(badc->adcQ, &buf, portMAX_DELAY);
	return buf;
}

void doneReadAdc(uint16_t *buf){
	vPortFree(buf);
}

//  ######  ########    ###    ######## ####  ######
// ##    ##    ##      ## ##      ##     ##  ##    ##
// ##          ##     ##   ##     ##     ##  ##
//  ######     ##    ##     ##    ##     ##  ##
//       ##    ##    #########    ##     ##  ##
// ##    ##    ##    ##     ##    ##     ##  ##    ##
//  ######     ##    ##     ##    ##    ####  ######


static void adcTask(void* pv){
	B_adcHandle_t *badc = pv;
	uint16_t *buf;
	for(;;){
		buf = pvPortMalloc(badc->numChannels*sizeof(uint8_t));
		HAL_ADC_Start_DMA(badc->hadc, buf, badc->numChannels);
		xSemaphoreTake(badc->adcSem, portMAX_DELAY);
		//HAL_ADC_Stop_DMA(badc->hadc);
		if(xQueueSendToBack(badc->adcQ, &buf, 0) != pdTRUE){
			vPortFree(buf);
		}
		vTaskDelay(2);
	}
}

//  ######     ###    ##       ##       ########     ###     ######  ##    ##
// ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
// ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
// ##       ##     ## ##       ##       ########  ##     ## ##       #####
// ##       ######### ##       ##       ##     ## ######### ##       ##  ##
// ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
//  ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##


void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc){
	for(int i = 0; i < NUM_ADCS; i++){
		if(hadcs[i] == hadc){
			xSemaphoreGiveFromISR(badcs[i]->adcSem, NULL);
			return;
		}
	}
}
