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

	GPIO_TypeDef* accelPort;
	uint16_t accelPin;

	GPIO_TypeDef* regenPort;
	uint16_t regenPin;

	SPI_HandleTypeDef* potSpiPtr;

	GPIO_TypeDef* vfmUpPort;
	uint16_t vfmUpPin;

	GPIO_TypeDef* vfmDownPort;
	uint16_t vfmDownPin;

	/* ==================== Private =============================*/
	TimerHandle_t vfmDownTimerHandle;
	TimerHandle_t vfmUpTimerHandle;

	uint32_t currentAccelValue;
	uint32_t currentRegenValue;

	uint32_t accelInputUpperBound;
	uint32_t regenInputUpperBound;

	//uint32_t turnOnPeriod;
	uint8_t isOn;
	uint8_t isForward;
	//uint32_t lastTurnOnTime;

	TimerHandle_t turnOnTimerHandle;

} MitsubaMotor;

// Note, you will need to set up the hardware perepherals yourself.
// This init function only sets up the rest.
MotorInterface* mitsubaMotor_init(MitsubaMotor* self);





#endif /* INC_MITSUBA_MOTOR_H_ */
