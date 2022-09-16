/*
 * mitsuba_motor.c
 *
 *  Created on: Jul 17, 2022
 *      Author: raymond
 */

#include <stdbool.h>

#include "mitsuba_motor.h"

enum motorPowerState {
	ECO,
	POWER
};


/* ================== Private function declaration (static) =========================*/
static void mitsubaMotor_eepromInit(MotorInterface* interface);
static void mitsubaMotor_vfmTimersInit(MotorInterface* interface);
static void mitsubaMotor_turnOnTimerInit(MotorInterface* interface);
static void mitsubaMotor_setInputUpperBounds(MotorInterface* interface, uint32_t accelInputUpperBound, uint32_t regenInputUpperBound);
static int mitsubaMotor_isOn(MotorInterface* interface);
static int mitsubaMotor_isForward(MotorInterface* interface);
static int mitsubaMotor_isAccel(MotorInterface* interface);
static int mitsubaMotor_isRegen(MotorInterface* interface);
static int mitsubaMotor_turnOn(MotorInterface* interface);
static int mitsubaMotor_turnOff(MotorInterface* interface);
static int mitsubaMotor_setEco(MotorInterface* interface);
static int mitsubaMotor_setPwr(MotorInterface* interface);
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
#define MCP4161_EEPROM_OFFSET   0x2
static void _MCP4161_Pot_Write(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr);
static void _MCP4161_Pot_Write_Internal(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr, bool eeprom);

/* =======================================================================*/



/* ============================== Implementations ========================*/

// Note, you will need to set up the hardware perepherals yourself.
// This init function only sets up the rest.
MotorInterface* mitsubaMotor_init(MitsubaMotor* self)
{

	// set up interface function pointers to point to private functions
	MotorInterface* interface = &self->interface;
	interface->implementation = self;
	interface->turnOn = mitsubaMotor_turnOn;
	interface->turnOff = mitsubaMotor_turnOff;
	interface->setForward = mitsubaMotor_setForward;
	interface->setEco = mitsubaMotor_setEco;
	interface->setPwr = mitsubaMotor_setPwr;
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

	mitsubaMotor_vfmTimersInit(&self->interface);
	mitsubaMotor_turnOnTimerInit(&self->interface);
	mitsubaMotor_setInputUpperBounds(&self->interface, MOTORINTERFACE_ACCEL_REGEN_INPUT_UPPERBOUND, MOTORINTERFACE_ACCEL_REGEN_INPUT_UPPERBOUND);
	self->isForward = 1;
	self->isOn = 0;
	self->ecoPwrState = ECO;

	// set motor pins to default state
	HAL_GPIO_WritePin(self->fwdRevPort, self->fwdRevPin, GPIO_PIN_SET); // FwdRev (high is forward)
	HAL_GPIO_WritePin(self->vfmUpPort, self->vfmUpPin, GPIO_PIN_SET); // VFM UP
	HAL_GPIO_WritePin(self->vfmDownPort, self->vfmDownPin, GPIO_PIN_SET); // VFM Down
	HAL_GPIO_WritePin(self->ecoPort, self->ecoPin, GPIO_PIN_SET); // ECO
	HAL_GPIO_WritePin(self->cs0AccelPort, self->cs0AccelPin, GPIO_PIN_SET); // CS0
	HAL_GPIO_WritePin(self->cs1RegenPort, self->cs1RegenPin, GPIO_PIN_SET); // CS1
	HAL_GPIO_WritePin(self->vfmResetPort, self->vfmResetPin, GPIO_PIN_SET); // VFM RESET
	HAL_GPIO_WritePin(self->MT3Port, self->MT3Pin, GPIO_PIN_SET); // MT3
	HAL_GPIO_WritePin(self->MT2Port, self->MT2Pin, GPIO_PIN_SET); // MT2
	HAL_GPIO_WritePin(self->MT1Port, self->MT1Pin, GPIO_PIN_SET); // MT1
	HAL_GPIO_WritePin(self->MT0Port, self->MT0Pin, GPIO_PIN_SET); // MT0

	mitsubaMotor_eepromInit(&self->interface);
	mitsubaMotor_setAccel(interface, 0);
	mitsubaMotor_setRegen(interface, 0);
	return interface;
}

static void mitsubaMotor_eepromInit(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;

	/* write to non-volative eeprom for each pot */
	_MCP4161_Pot_Write_Internal(0, self->cs0AccelPort, self->cs0AccelPin, self->potSpiPtr, true);
	_MCP4161_Pot_Write_Internal(0, self->cs1RegenPort, self->cs1RegenPin, self->potSpiPtr, true);

	/* need to wait for t_wc (max. 10ms) */
	HAL_Delay(10);
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
		if (!xTimerIsTimerActive(self->turnOnTimerHandle)) {
			HAL_GPIO_WritePin(self->mainPort, self->mainPin, GPIO_PIN_RESET);
			return (xTimerStart(self->turnOnTimerHandle, 0));
		} else {
			return 0;
		}
	}
	return 0;
}

static void mitsubaMotor_turnOnTimerCallback(TimerHandle_t xTimer)
{
	MitsubaMotor* self = (MitsubaMotor*)pvTimerGetTimerID(xTimer);
	if (HAL_GPIO_ReadPin(self->mainPort, self->mainPin) == GPIO_PIN_RESET) {
		// only set isOn to 1 if the motor has not been turned off during the 5 seconds
		self->isOn = 1;
	}
}

static void mitsubaMotor_turnOnTimerInit(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	self->turnOnTimerHandle = xTimerCreate("TurnOnTimer", pdMS_TO_TICKS(MITSUBA_MOTOR_TURN_ON_PERIOD), pdFALSE, self, mitsubaMotor_turnOnTimerCallback);
	if (self->turnOnTimerHandle == NULL) {
		configASSERT(pdFAIL);
	}

}

static int mitsubaMotor_turnOff(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	HAL_GPIO_WritePin(self->mainPort, self->mainPin, GPIO_PIN_SET);
	self->isOn = 0;

	if (interface->isAccel(interface)) interface->setAccel(interface, 0);
	if (interface->isRegen(interface)) interface->setRegen(interface, 0);

	return 1;
}

static int mitsubaMotor_setForward(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (mitsubaMotor_isOn(interface)) {
		HAL_GPIO_WritePin(self->fwdRevPort, self->fwdRevPin, GPIO_PIN_RESET);
		self->isForward = 1;
		return 1;
	}
	return 0;
}

static int mitsubaMotor_setEco(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (mitsubaMotor_isOn(interface) && self->ecoPwrState != ECO) {
		HAL_GPIO_WritePin(self->ecoPort, self->ecoPin, GPIO_PIN_SET); //3.3V on MCMB turns off current on optoisolator, which opens optoisolator output (MC^2) and goes to ECO mode
		self->ecoPwrState = ECO;
		return 1;
	}
	return 0;
}

static int mitsubaMotor_setPwr(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (mitsubaMotor_isOn(interface) && self->ecoPwrState != POWER) {
		HAL_GPIO_WritePin(self->ecoPort, self->ecoPin, GPIO_PIN_RESET); //0V on MCMB turns on current on optoisolator, which closes optoisolator output (MC^2) and goes to PWR mode
		self->ecoPwrState = POWER;
		return 1;
	}
	return 0;
}

static int mitsubaMotor_setReverse(MotorInterface* interface)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (mitsubaMotor_isOn(interface)) {
		HAL_GPIO_WritePin(self->fwdRevPort, self->fwdRevPin, GPIO_PIN_SET);
		self->isForward = 0;
		return 1;
	}
	return 0;
}


static int mitsubaMotor_setAccel(MotorInterface* interface, uint32_t val)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (val != 0 && (!mitsubaMotor_isOn(interface) || mitsubaMotor_isRegen(interface))) {
		return 0;
	}
	uint16_t wiperValue;
	if (self->accelInputUpperBound != MCP4161_MAX_WIPER_VALUE) {
		// map to 0 - MCP4161_MAX_WIPER_VALUE
		wiperValue = val / self->accelInputUpperBound * MCP4161_MAX_WIPER_VALUE;
	} else {
		wiperValue = val;
	}
	_MCP4161_Pot_Write(wiperValue, self->cs0AccelPort, self->cs0AccelPin, self->potSpiPtr);
	self->currentAccelValue = wiperValue;
	return 1;
}

static int mitsubaMotor_setRegen(MotorInterface* interface, uint32_t val)
{
	MitsubaMotor* self = (MitsubaMotor*)interface->implementation;
	if (val != 0 && (!mitsubaMotor_isOn(interface) || mitsubaMotor_isAccel(interface))) {
		return 0;
	}
	uint16_t wiperValue;
	if (self->regenInputUpperBound != MCP4161_MAX_WIPER_VALUE) {
		// map to 0 - MCP4161_MAX_WIPER_VALUE
		wiperValue = val / self->regenInputUpperBound * MCP4161_MAX_WIPER_VALUE;
	} else {
		wiperValue = val;
	}
	_MCP4161_Pot_Write(wiperValue, self->cs1RegenPort, self->cs1RegenPin, self->potSpiPtr);
	self->currentRegenValue = wiperValue;
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
		configASSERT(pdFAIL);
	}
	self->vfmDownTimerHandle = xTimerCreate("vfmDownTimer", pdMS_TO_TICKS(200), pdFALSE, self, mitsubaMotor_vfmDownCallback);
	if (self->vfmDownTimerHandle == NULL) {
		configASSERT(pdFAIL);
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
static void _MCP4161_Pot_Write(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr)
{
	_MCP4161_Pot_Write_Internal(wiperValue, CSPort, CSPin, hspiPtr, false);
}

static void _MCP4161_Pot_Write_Internal(uint16_t wiperValue, GPIO_TypeDef *CSPort, uint16_t CSPin, SPI_HandleTypeDef *hspiPtr, bool eeprom)
{
	if (wiperValue > MCP4161_MAX_WIPER_VALUE) {
		wiperValue = MCP4161_MAX_WIPER_VALUE; // Since the highest wiperValue is 256
	}
	uint8_t ninethDataBit = (wiperValue >> 8) & 0b1;
	uint8_t potAddress = 0b0000 + (eeprom ? MCP4161_EEPROM_OFFSET : 0);
	uint8_t writeCommand = 0b00;

	uint8_t commandByte  = (potAddress << 4) | (writeCommand << 2) | ninethDataBit;
	uint8_t dataByte = wiperValue & 0xFF;

	uint8_t fullCommand[2] = {commandByte, dataByte};

	// Transmit using SPI
	HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspiPtr, fullCommand, sizeof(fullCommand), 100);
	HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);
}
