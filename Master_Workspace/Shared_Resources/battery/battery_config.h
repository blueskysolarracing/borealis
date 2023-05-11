#ifndef __BATTERY_CONFIG_H__
#define __BATTERY_CONFIG_H__

#define HV_BATT_OC_DISCHARGE 	45.0 	//Should be set to 45.0A
#define HV_BATT_OC_CHARGE 		30.0 	//Should be set to 30.0A
#define HV_BATT_OV_THRESHOLD 	4.20	//Should be set to 4.20V
#define HV_BATT_UV_THRESHOLD 	2.50 	//Should be set to 2.50V
#define HV_BATT_OT_THRESHOLD 	65.0 	//Should be set to 65.0C


#define NUM_CELLS_PER_MODULE 	5
#define NUM_BMS_MODULES			6 		//Number of BMS modules to read from
#define NUM_BATT_CELLS 			(NUM_BMS_MODULES*NUM_CELLS_PER_MODULE) 		//Number of series parallel groups in battery pack
#define NUM_TEMP_SENSORS_PER_MODULE 3
#define NUM_BATT_TEMP_SENSORS 	(NUM_TEMP_SENSORS_PER_MODULE*NUM_BMS_MODULES) 	//Number of temperature sensors in battery pack


#endif // __BATTERY_CONFIG_H__