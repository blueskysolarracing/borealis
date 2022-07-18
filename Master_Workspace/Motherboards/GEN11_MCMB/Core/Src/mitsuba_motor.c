/*
 * mitsuba_motor.c
 *
 *  Created on: Jul 17, 2022
 *      Author: raymond
 */

#include "mitsuba_motor.h"

/* ================== Private functions (static) =========================*/
static void mitsubaMotor_vfmTimersInit(MotorInterface* interface);
static void mitsubaMotor_turnOnTimerInit(MotorInterface* interface);
static void mitsubaMotor_setInputUpperBounds(MotorInterface* interface, uint32_t accelInputUpperBound, uint32_t regenInputUpperBound);
static int mitsubaMotor_isOn(MotorInterface* interface);
static int mitsubaMotor_isForward(MotorInterface* interface);
static int mitsubaMotor_isAccel(MotorInterface* interface);
static int mitsubaMotor_isRegen(MotorInterface* interface);
static int mitsubaMotor_turnOn(MotorInterface* interface);
static int mitsubaMotor_turnOff(MotorInterface* interface);
static int mitsubaMotor_setForward(MotorInterface* interface);
static int mitsubaMotor_setReverse(MotorInterface* interface);
static int mitsubaMotor_setAccel(MotorInterface* interface, uint32_t val);
static int mitsubaMotor_setRegen(MotorInterface* interface, uint32_t val);
static int mitsubaMotor_vfmUp(MotorInterface* interface);
static int mitsubaMotor_vfmDown(MotorInterface* interface);
static int mitsubaMotor_getTurnOnPeriod(MotorInterface* interface, uint32_t turnOnPeriod);

/*
 * Function to set the wiper position of the MCP4146 potentiometer on the MC^2.
 * Pass in a wiperValue (range from 0 - 256) to set the wiper position of the potentiometer (which ranges from 0 - 256).
 * Must also pass in the appropriate GPIO port and pins for chip select and the address of SPI handle.
 *
 * Note: In cubeMx make sure SPI CLK is below 10Mhz
 * 		And, configure SPI to send MSB first, and send 8 bits at a time
*/
#define MCP4161_MAX_WIPER_VALUE 255 // actually, it is 256, but we don't really need that 1 extra value, so to prevent mapping, we go for 255
static void _MCP4161_Pot_Write(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr);

/* =======================================================================*/


// Note, you will need to set up the hardware perepherals yourself.
// This init function only sets up the rest.
MotorInterface* mitsubaMotor_init(MitsubaMotor* self)
{
	mitsubaMotor_vfmTimersInit(&self->interface);
	mitsubaMotor_turnOnTimerInit(&self->interface);
	mitsubaMotor_setInputUpperBounds(&self->interface, MOTORINTERFACE_ACCEL_REGEN_INPUT_UPPERBOUND, MOTORINTERFACE_ACCEL_REGEN_INPUT_UPPERBOUND);
	self->isForward = 1;
	self->isOn = 0;

	// set up interface
	MotorInterface* interface = &self->interface;
	interface->implementation = self;
	interface->turnOn = mitsubaMotor_turnOn;
	interface->turnOff = mitsubaMotor_turnOff;
	interface->setForward = mitsubaMotor_setForward;
	interface->setReverse = mitsubaMotor_setReverse;
	interface->setAccel = mitsubaMotor_setAccel;
	interface->setRegen = mitsubaMotor_setRegen;
	interface->gearUp = mitsubaMotor_vfmUp;
	interface->gearDown = mitsubaMotor_vfmDown;

	interface->getTurnOnPeriod = mitsubaMotor_getTurnOnPeriod;
	interface->isOn = mitsubaMotor_isOn;
	interface->isForward = mitsubaMotor_isForward;
	interface->isAccel = mitsubaMotor_isAccel;
	interface->isRegen = mitsubaMotor_isRegen;
}


static int mitsubaMotor_isOn(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	return self->isOn;
}
static int mitsubaMotor_isForward(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	return self->isForward;
}
static int mitsubaMotor_isAccel(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	return self->currentAccelValue > 0 ? 1 : 0;
}

static int mitsubaMotor_isRegen(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	return self->currentRegenValue > 0 ? 1 : 0;
}


static int mitsubaMotor_turnOn(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (!mitsubaMotor_isOn(interface)) {
		HAL_GPIO_WritePin(self->mainPort, self->mainPin, GPIO_PIN_RESET);
		return (xTimerStart(self->turnOnTimerHandle, 0));
	}
	return 0;
}

static void mitsubaMotor_turnOnTimerCallback(TimerHandle_t xTimer)
{
	MitsubaMotor* self = (MitsubaMotor*)pvTimerGetTimerID(xTimer);
	self->isOn = 1;
}

static void mitsubaMotor_turnOnTimerInit(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	self->turnOnTimerHandle = xTimerCreate("TurnOnTimer", pdMS_TO_TICKS(MITSUBA_MOTOR_TURN_ON_PERIOD), pdFALSE, self, mitsubaMotor_turnOnTimerCallback);
	if (self->turnOnTimerHandle == NULL) {
		configAssert(pdFAIL);
	}

}

static int mitsubaMotor_turnOff(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	HAL_GPIO_WritePin(self->mainPort, self->mainPin, GPIO_PIN_SET);
	self->isOn = 0;
	return 1;
}

static int mitsubaMotor_setForward(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (mitsubaMotor_isOn(interface)) {
		HAL_GPIO_WritePin(self->fwdRevPort, self->fwdRevPin, GPIO_PIN_SET);
		self->isForward = 1;
		return 1;
	}
	return 0;
}

static int mitsubaMotor_setReverse(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (mitsubaMotor_isOn(interface)) {
		HAL_GPIO_WritePin(self->fwdRevPort, self->fwdRevPin, GPIO_PIN_RESET);
		self->isForward = 0;
		return 1;
	}
	return 0;
}


static int mitsubaMotor_setAccel(MotorInterface* interface, uint32_t val)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (!mitsubaMotor_isOn(interface) || mitsubaMotor_isRegen(interface)) {
		return 0;
	}
	uint16_t wiperValue;
	if (self->accelInputUpperBound != MCP4161_MAX_WIPER_VALUE) {
		// map to 0 - MCP4161_MAX_WIPER_VALUE
		wiperValue = val / self->accelInputUpperBound * MCP4161_MAX_WIPER_VALUE;
	} else {
		wiperValue = val;
	}
	_MCP4161_Pot_Write(wiperValue, self->accelPort, self->accelPin, self->potSpiPtr);
	self->currentAccelValue = wiperValue;
	return 1;
}

static int mitsubaMotor_setRegen(MotorInterface* interface, uint32_t val)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (!mitsubaMotor_isOn(interface) || mitsubaMotor_isAccel(interface)) {
		return 0;
	}
	uint16_t wiperValue;
	if (self->regenInputUpperBound != MCP4161_MAX_WIPER_VALUE) {
		// map to 0 - MCP4161_MAX_WIPER_VALUE
		wiperValue = val / self->regenInputUpperBound * MCP4161_MAX_WIPER_VALUE;
	} else {
		wiperValue = val;
	}
	_MCP4161_Pot_Write(wiperValue, self->regenPort, self->regenPin, self->potSpiPtr);
	self->currentAccelValue = wiperValue;
	return 1;
}


static int mitsubaMotor_vfmUp(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (xTimerIsTimerActive(self->vfmUpTimerHandle) == pdTRUE) {
		return 0; // Previous timer not finished
	}
	if (xTimerStart(self->vfmUpTimerHandle, 0) == pdFAIL) {
		return 0;
	} else {
		HAL_GPIO_WritePin(self->vfmUpPort, self->vfmUpPin, GPIO_PIN_RESET);
		return 1;
	}
}

static void mitsubaMotor_vfmUpCallback(TimerHandle_t xTimer)
{
	MitsubaMotor* self = (MitsubaMotor*)pvTimerGetTimerID(xTimer);
	HAL_GPIO_WritePin(self->vfmUpPort, self->vfmUpPin, GPIO_PIN_SET);
}

static int mitsubaMotor_vfmDown(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (xTimerIsTimerActive(self->vfmDownTimerHandle) == pdTRUE) {
		return 0; // Previous timer not finished
	}
	if (xTimerStart(self->vfmDownTimerHandle, 0) == pdFAIL) {
		return 0;
	} else {
		HAL_GPIO_WritePin(self->vfmDownPort, self->vfmDownPin, GPIO_PIN_RESET);
		return 1;
	}
}

static void mitsubaMotor_vfmDownCallback(TimerHandle_t xTimer)
{
	MitsubaMotor* self = (MitsubaMotor*)pvTimerGetTimerID(xTimer);
	HAL_GPIO_WritePin(self->vfmDownPort, self->vfmDownPin, GPIO_PIN_SET);
}

static void mitsubaMotor_vfmTimersInit(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	self->vfmUpTimerHandle = xTimerCreate("vfmUpTimer", pdMS_TO_TICKS(200), pdFALSE, self, mitsubaMotor_vfmUpCallback);
	if (self->vfmUpTimerHandle == NULL) {
		configAssert(pdFAIL);

	}
	self->vfmDownTimerHandle = xTimerCreate("vfmDownTimer", pdMS_TO_TICKS(200), pdFALSE, self, mitsubaMotor_vfmDownCallback);
	if (self->vfmDownTimerHandle == NULL) {
		configAssert(pdFAIL);
	}
}

void mitsubaMotor_setInputUpperBounds(MotorInterface* interface, uint32_t accelInputUpperBound, uint32_t regenInputUpperBound)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	self->accelInputUpperBound = accelInputUpperBound;
	self->regenInputUpperBound = regenInputUpperBound;
}

static int mitsubaMotor_getTurnOnPeriod(MotorInterface* interface, uint32_t turnOnPeriod)
{
	return MITSUBA_MOTOR_TURN_ON_PERIOD;
}


/*
 * Function to set the wiper position of the MCP4146 potentiometer on the MC^2.
 * Pass in a wiperValue (range from 0 - 256) to set the wiper position of the potentiometer (which ranges from 0 - 256).
 * Must also pass in the appropriate GPIO port and pins for chip select and the address of SPI handle.
 *
 * Note: In cubeMx make sure SPI CLK is below 10Mhz
 * 		And, configure SPI to send MSB first, and send 8 bits at a time
*/
#define MCP4161_MAX_WIPER_VALUE 255 // actually, it is 256, but we don't really need that 1 extra value, so to prevent mapping, we go for 255
static void _MCP4161_Pot_Write(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr)
{
	if (wiperValue > MCP4161_MAX_WIPER_VALUE) {
		wiperValue = MCP4161_MAX_WIPER_VALUE; // Since the highest wiperValue is 256
	}
	uint8_t ninethDataBit = (wiperValue >> 8) & 0b1;
	uint8_t potAddress = 0b0000;
	uint8_t writeCommand = 0b00;

	uint8_t commandByte  = (potAddress << 4) | (writeCommand << 2) | ninethDataBit;
	uint8_t dataByte = wiperValue & 0xFF;

	uint8_t fullCommand[2] = {commandByte, dataByte};

	// Transmit using SPI
	HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspiPtr, fullCommand, sizeof(fullCommand), 100);
	HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);
}
