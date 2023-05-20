#ifndef __BATTERY_CONFIG_H__
#define __BATTERY_CONFIG_H__

#define HV_BATT_OC_DISCHARGE 	(45.0f) 	//Should be set to 45.0A
#define HV_BATT_OC_CHARGE 		(30.0f) 	//Should be set to 30.0A
#define HV_BATT_OV_THRESHOLD 	(4.20f)	    //Should be set to 4.20V
#define HV_BATT_UV_THRESHOLD 	(2.50f)	    //Should be set to 2.50V
#define HV_BATT_OT_THRESHOLD 	(65.0f) 	//Should be set to 65.0C


#define NUM_CELLS_PER_MODULE 	(5)
#define NUM_BMS_MODULES			(6) 		//Number of BMS modules to read from
#define NUM_BATT_CELLS 			(NUM_BMS_MODULES*NUM_CELLS_PER_MODULE) 		//Number of series parallel groups in battery pack
#define NUM_TEMP_SENSORS_PER_MODULE (3)
#define NUM_BATT_TEMP_SENSORS 	(NUM_TEMP_SENSORS_PER_MODULE*NUM_BMS_MODULES) 	//Number of temperature sensors in battery pack

// Note: -1000.0f is used as an initial value. If the value is still -1000.0f after program runs, it means the corresponding cell, termister is unconnected
#define BATTERY_CELL_VOLTAGES_INITIAL_VALUE     (-1000.0f)
#define BATTERY_CELL_VOLTAGES_FAKE_VALUE        (0.0f)
#define BATTERY_TEMPERATURES_INITIAL_VALUE      (-1000.0f)
#define BATTERY_SOC_INITIAL_VALUE               (-1000.0f)


#endif // __BATTERY_CONFIG_H__
