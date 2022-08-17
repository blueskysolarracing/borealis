/*
 * Helper functions used on 1+ motherboard
 */
#include "main.h"

#define PRECHARGE_TIME 500 //Delay to wait after closing pre-charge relay before closing other relays
#define DISCHARGE_TIME 1000 //Delay to wait after setting dischange pin high to discharge bus capacitance
#define ACTUATION_DELAY 50 //Delay to wait after opening/closing relay (Gigavac GX14 has max. 20ms actuation time: https://www.gigavac.com/sites/default/files/catalog/spec_sheet/gx14.pdf)

struct relay_periph{
	GPIO_TypeDef* PRE_SIG_GPIO_Port;
	uint16_t PRE_SIG_Pin;

	GPIO_TypeDef* DISCHARGE_GPIO_Port;
	uint16_t DISCHARGE_Pin;

	GPIO_TypeDef* ON_SIG_GPIO_Port;
	uint16_t ON_SIG_Pin;

	GPIO_TypeDef* GND_SIG_GPIO_Port;
	uint16_t GND_SIG_Pin;

	uint8_t battery_relay_state;
	uint8_t array_relay_state;
};

void open_relays(struct relay_periph* relay);
void close_relays(struct relay_periph* relay);
