#ifndef __BATTERY_CONFIG_H__
#define __BATTERY_CONFIG_H__

#define HV_BATT_OC_THRESHOLD 	(60.0f) 	//Should be set to 60.0A
#define HV_BATT_OV_THRESHOLD 	(4.25f)	    //Should be set to 4.25V
#define HV_BATT_UV_THRESHOLD 	(2.50f)	    //Should be set to 2.50V
#define HV_BATT_OT_THRESHOLD 	(60.0f) 	//Should be set to 60.0C
#define HV_BATT_UT_THRESHOLD 	(0.0f) 		//Should be set to 0.0C


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

#define MAX_ALLOWD_OVERCURRENT_TIME (7000) // 7 seconds
#define MOTOR_OT_THRESHOLD (180.0f) // 180.0C
#define MOTOR_OT_WARNING_THRESHOLD (100.0f) // 100.0C

#define HV_BATT_OV_DEBOUNCE_TIME 3000 // 3 seconds
#define HV_BATT_UV_DEBOUNCE_TIME 3000 // 3 seconds

// deprecated
enum BATTERY_FAULT_TYPE {
	BATTERY_FAULT_OVERTEMPERATURE,
	BATTERY_FAULT_UNDERTEMPERATURE,
	BATTERY_FAULT_OVERVOLTAGE,
	BATTERY_FAULT_UNDERVOLTAGE,
	BATTERY_FAULT_OVERCURRENT,
	BATTERY_FAULT_NONE,
};

#endif // __BATTERY_CONFIG_H__
