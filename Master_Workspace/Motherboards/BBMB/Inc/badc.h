#include "main.h"
#include "cmsis_os.h"

typedef struct{
	uint32_t val;
	uint32_t ADC_CHANNEL;
}B_adcQEntry_t;

// not __attribute__((__packed__))
typedef struct{
	ADC_HandleTypeDef* 		hadc;
	QueueHandle_t 			adcQ;
	uint16_t* 				adcBuf;
	TaskHandle_t			adcTask;
	uint8_t					numChannels;
	SemaphoreHandle_t 		adcSem;
}B_adcHandle_t;

B_adcHandle_t* B_adcStart(ADC_HandleTypeDef* hadc, uint8_t numChannels);
uint16_t* readAdc(B_adcHandle_t *badc);
void doneReadAdc(uint16_t *buf);

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc);
