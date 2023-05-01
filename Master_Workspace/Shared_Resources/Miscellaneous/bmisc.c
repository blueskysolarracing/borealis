#include "bmisc.h"
#include "main.h"
#include "cmsis_os.h"
#include "protocol_ids.h"

/*
 * Need to initialize relayPeriph in main.c like so:
 *
 * struct relayPeriph relay;
 * relay.PRE_SIG_GPIO_Port = Port name in .ioc
 * relay.PRE_SIG_Pin = Pin name in .ioc
 *
 * relay.DISCHARGE_GPIO_Port = Port name in .ioc
 * relay.DISCHARGE_Pin = Pin name in .ioc
 *
 * relay.ON_SIG_GPIO_Port = Port name in .ioc
 * relay.ON_SIG_Pin = Pin name in .ioc
 *
 * relay.GND_SIG_GPIO_Port = Port name in .ioc
 * relay.GND_SIG_Pin = Pin name in .ioc
 *
 * and call open_relays(&relay); or close_relays(&relay);
 *
 * Note that this will block the current task since it's a delay (but not other tasks)
 */

void open_relays(struct relay_periph* relay){
	// Note we cannot enable discharge in this function as only BBMB controls discharge

	//Open the power relays around the battery to disconnect the battery from the HV system
	HAL_GPIO_WritePin(relay->PRE_SIG_GPIO_Port, relay->PRE_SIG_Pin, GPIO_PIN_RESET); //Make sure the precharge relay is open (should be at this point)
	osDelay(ACTUATION_DELAY);

	HAL_GPIO_WritePin(relay->ON_SIG_GPIO_Port, relay->ON_SIG_Pin, GPIO_PIN_RESET); //Open high-side relay
	osDelay(ACTUATION_DELAY);

	HAL_GPIO_WritePin(relay->GND_SIG_GPIO_Port, relay->GND_SIG_Pin, GPIO_PIN_RESET); //Open low-side relay
	osDelay(ACTUATION_DELAY);
}

void close_relays(struct relay_periph* relay){
	//Close the power relays (sequentially) around the battery to connect the battery to the HV system
	HAL_GPIO_WritePin(relay->DISCHARGE_GPIO_Port, relay->DISCHARGE_Pin, GPIO_PIN_SET); //Make sure battery discharge circuit is disabled
	osDelay(ACTUATION_DELAY);

	HAL_GPIO_WritePin(relay->GND_SIG_GPIO_Port, relay->GND_SIG_Pin, GPIO_PIN_SET); //Close low-side relay
	osDelay(ACTUATION_DELAY);

	HAL_GPIO_WritePin(relay->PRE_SIG_GPIO_Port, relay->PRE_SIG_Pin, GPIO_PIN_SET); //Close pre-charge relay to safely charge bus capacitance and avoid high dI/dt
	osDelay(PRECHARGE_TIME);

	HAL_GPIO_WritePin(relay->ON_SIG_GPIO_Port, relay->ON_SIG_Pin, GPIO_PIN_SET); //Close high-side relay
	osDelay(ACTUATION_DELAY + 500); //500ms added to prevent inrush current from tripping PSM measurements

	HAL_GPIO_WritePin(relay->PRE_SIG_GPIO_Port, relay->PRE_SIG_Pin, GPIO_PIN_RESET); //Open pre-charge relay
}
