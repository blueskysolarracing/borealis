/*
 * mitsuba_motor.h
 *
 *  Created on: Jul 12, 2022
 *      Author: raymond
 */



#ifndef INC_MITSUBA_MOTOR_H_
#define INC_MITSUBA_MOTOR_H_
#include "motor_interface.h"

#include "FreeRTOSConfig.h"
#include "cmsis_os.h"


#include "main.h"

// MitsubaMotor Implements MotorInterface
#define MITSUBA_MOTOR_TURN_ON_PERIOD 5000 // takes 5000 ms for this motor to turn on
typedef struct MitsubaMotor {

	MotorInterface interface;

	/* =========== peripherals you need to initialize ============ */
	GPIO_TypeDef* mainPort;
	uint16_t mainPin;

	GPIO_TypeDef* fwdRevPort;
	uint16_t fwdRevPin;

	GPIO_TypeDef* cs0AccelPort;
	uint16_t cs0AccelPin;

	GPIO_TypeDef* cs1RegenPort;
	uint16_t cs1RegenPin;

	SPI_HandleTypeDef* potSpiPtr;

	GPIO_TypeDef* vfmUpPort;
	uint16_t vfmUpPin;

	GPIO_TypeDef* vfmDownPort;
	uint16_t vfmDownPin;

	GPIO_TypeDef* vfmResetPort;
	uint16_t vfmResetPin;

	GPIO_TypeDef* ecoPort;
	uint16_t ecoPin;

	GPIO_TypeDef* MT3Port;
	uint16_t MT3Pin;

	GPIO_TypeDef* MT2Port;
	uint16_t MT2Pin;

	GPIO_TypeDef* MT1Port;
	uint16_t MT1Pin;

	GPIO_TypeDef* MT0Port;
	uint16_t MT0Pin;


	/* ==================== Private =============================*/
//	TimerHandle_t vfmDownTimerHandle;
//	TimerHandle_t vfmUpTimerHandle;
	//////////
	//BaseType_t gearChangeThreadHandleStatus ;
	TaskHandle_t vfmThreadHandle;
	QueueHandle_t vfmQueueHandle;
	uint8_t vfmQueueLen = 20;

	uint32_t currentAccelValue;
	uint32_t currentRegenValue;

	uint32_t accelInputUpperBound;
	uint32_t regenInputUpperBound;

	//uint32_t turnOnPeriod;
	uint8_t isOn;
	uint8_t isForward;
	uint8_t ecoPwrState;
	//uint32_t lastTurnOnTime;

	TimerHandle_t turnOnTimerHandle;

} MitsubaMotor;


// Note, you will need to set up the hardware perepherals yourself.
// This init function only sets up the rest.
MotorInterface* mitsubaMotor_init(MitsubaMotor* self);


#endif /* INC_MITSUBA_MOTOR_H_ */
