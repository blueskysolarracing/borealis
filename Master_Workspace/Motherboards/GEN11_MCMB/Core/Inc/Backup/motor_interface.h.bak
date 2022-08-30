/*
 * motor_interface.h
 *
 *  Created on: Jul 11, 2022
 *      Author: raymond
 */
#ifndef MOTORINTERFACE_H
#define MOTORINTERFACE_H
#include "stdint.h"
/*
 *	The motor interface defines a set of required functions for the motor
 * 	A derived structure should implement these functions depending on the hardware and motor used
 * 	Important: All functions must be implemented to be non-blocking. If needed, use threads or software timers to complete delay-related tasks
 */
#define MOTORINTERFACE_ACCEL_REGEN_INPUT_UPPERBOUND 255
typedef struct MotorInterface {

	void* implementation; // Points to the structure which implements this interface

	/* =============== MOTOR CONTROL FUNCTIONS BEGIN ====================*/
	/* The following functions should return 1 if commands to the motor are sent, 0 if not sent
	 * Examples for returning 0, when commands are not sent, can include:
	 * 1. Trying to set Accel too quickly after turning on the motor, without waiting for the turn-on process to complete
	 * 2. Trying to set Accel while already in Regen, or vice versa
	 * 3. Switching gear while the previous gear switch is not finished
	 */
	int (*turnOn)(struct MotorInterface* self);
	int (*turnOff)(struct MotorInterface* self);
	int (*setEco)(struct MotorInterface* self);
	int (*setPwr)(struct MotorInterface* self);
	int (*setForward)(struct MotorInterface* self);
	int (*setReverse)(struct MotorInterface* self);

	/* Note: The input to setAccel() and setRegen() should be a value between 0 - 255
	 * Then, in your implementation of setAccel(), you are responsible for mapping this 0 - 200 value...
	 * ...to whatever value is required by the motor control system (such as 0 - 100)
	 */

	int (*setAccel)(struct MotorInterface* self, uint32_t val);
	int (*setRegen)(struct MotorInterface* self, uint32_t val);

	// Might not be necessary for some motors
	int (*gearUp) (struct MotorInterface* self);
	int (*gearDown) (struct MotorInterface* self);
	/* =============== MOTOR CONTROL FUNCTIONS END ====================*/


	/* ==================== get state of motor ===========================*/
	// TurnOnPeriod: The amount of time (ms) it takes for motor complete its turn-on process (set to 5000 ms for Mitsuba)
	// This could be necessary for some applications to know, as some motors can break if you send commands to it before while it is in the process of turning on
	int (*getTurnOnPeriod)(struct MotorInterface* self, uint32_t turnOnPeriod);
	int (*isOn)(struct MotorInterface* self);
	int (*isForward)(struct MotorInterface* self);
	int (*isAccel)(struct MotorInterface* self);
	int (*isRegen)(struct MotorInterface* self);

} MotorInterface;




#endif



