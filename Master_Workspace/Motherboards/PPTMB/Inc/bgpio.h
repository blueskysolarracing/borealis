#include "main.h"
#include "cmsis_os.h"
#ifndef __BGPIO_H
#define __BGPIO_H
typedef struct{
	GPIO_TypeDef *gpio;
	uint16_t pin;
} B_gpioPin_t;

static void B_gpioSet(B_gpioPin_t *pin);
static void B_gpioReset(B_gpioPin_t *pin);
static GPIO_PinState B_gpioRead(B_gpioPin_t *pin);



#endif
