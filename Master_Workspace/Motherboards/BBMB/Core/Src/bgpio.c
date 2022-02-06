#include "bgpio.h"

// ########  ######## ########
// ##     ## ##       ##     ##
// ##     ## ##       ##     ##
// ########  ######   ########
// ##        ##       ##
// ##        ##       ##
// ##        ##       ##

static void B_gpioSet(B_gpioPin_t *pin);
static void B_gpioReset(B_gpioPin_t *pin);
static GPIO_PinState B_gpioRead(B_gpioPin_t *pin);

//  ######  ########    ###    ######## ####  ######
// ##    ##    ##      ## ##      ##     ##  ##    ##
// ##          ##     ##   ##     ##     ##  ##
//  ######     ##    ##     ##    ##     ##  ##
//       ##    ##    #########    ##     ##  ##
// ##    ##    ##    ##     ##    ##     ##  ##    ##
//  ######     ##    ##     ##    ##    ####  ######

static void B_gpioSet(B_gpioPin_t *pin){
	HAL_GPIO_WritePin(pin->gpio, pin->pin, GPIO_PIN_SET);
}

static void B_gpioReset(B_gpioPin_t *pin){
	HAL_GPIO_WritePin(pin->gpio, pin->pin, GPIO_PIN_RESET);
}

static GPIO_PinState B_gpioRead(B_gpioPin_t *pin){
	return HAL_GPIO_ReadPin(pin->gpio, pin->pin);
}
