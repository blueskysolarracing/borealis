

#ifndef __PROTOCOLIDS_H
#define __PROTOCOLIDS_H

/* ===== Sender IDs ===== */
enum Sender_ID {
	SENDER_ID_NOT_USED, // HAS VALUE OF ZERO
	BBMB_ID,
	PPTMB_ID,
	MCMB_ID,
	DCMB_ID,
	CHASE_ID,
	BMS_ID
};


/* ===== Data IDs ===== */
enum BBMB_Data_ID {
	BBMB_BUS_METRICS_ID,
	BBMB_FAULT_STATE_ID,
	BBMB_BSD_ID, // not used
	BBMB_BMS_MCU_STATUS_ID,   // not used
	BBMB_UNUSED_ID_0x4,
	BBMB_BMS_DATA_REQUEST_ID,
	BBMB_RELAYS_STATE_ID,
	
	BBMB_LP_BUS_METRICS_ID = 	0x0D,  // not used
	BBMB_CORE_TEMP_ID = 		0x0E,  // not used
	BBMB_HEARTBEAT_ID = 		0x0F
};

enum PPTMB_DATA_ID {
	PPTMB_BUS_METRICS_ID,
	PPTMB_PPT_METRICS_ID,
	
	PPTMB_RELAYS_STATE_ID = 	0x06,
	PPTMB_LP_BUS_METRICS_ID = 	0x0D,
	PPTMB_CORE_TEMP_ID = 		0x0E,
	PPTMB_HEARTBEAT_ID = 		0x0F
};

enum MCMB_Data_ID {
	MCMB_BUS_METRICS_ID,
	MCMB_CAR_SPEED_ID,
	MCMB_MOTOR_TEMPERATURE_ID,
	MCMB_SUPP_BATT_VOLTAGE_ID,
	
	MCMB_LP_BUS_METRICS_ID = 	0x0D,
	MCMB_CORE_TEMP_ID = 		0x0E,
	MCMB_HEARTBEAT_ID = 		0x0F
};

enum DCMB_Data_ID {
	DCMB_MC2_STATE_ID, //To be deprecated
	DCMB_BBOX_STARTUP_ID,
	DCMB_PPTBOX_STARTUP_ID,
	DCMB_LIGHTCONTROL_ID,
	DCMB_STEERING_WHEEL_ID,
	DCMB_MOTOR_CONTROL_STATE_ID,
	DCMB_RELAYS_STATE_ID,
	DCMB_PEDALS_ANGLE_ID,
	DCMB_SIDE_PANEL_ID,

	DCMB_LP_BUS_METRICS_ID = 	0x0D,
	DCMB_CORE_TEMP_ID = 		0x0E,
	DCMB_HEARTBEAT_ID = 		0x0F
};

enum Chase_Data_ID {
	CHASE_HEARTBEAT_REQ_ID,
	CHASE_SOFT_RESET_REQ_ID,
	CHASE_SET_FREQUENCY_ID,
	CHASE_LIGHTCONTROL_ID,
	CHASE_SET_RTC_ID,
	CHASE_CRUISE_PI_GAIN_ID,
	CHASE_MESSAGE_ID,

	CHASE_HEARTBEAT_ID = 		0x0F
};

enum BMS_Data_ID {
	BMS_CELL_TEMP_ID = 			0x07,
	BMS_CELL_VOLT_ID = 			0x08,
	BMS_CELL_SOC_ID = 			0x09,

	BMS_HEARTBEAT_ID = 			0x0A
};

enum BMS_Error_Code {
	BMS_OV = 0x01, //Cell overvoltage
	BMS_UV = 0x02, //Cell undervoltage
	BMS_OT = 0x03, //Cell overtemperature
	BMS_LTC6810_UNRESPONSIVE = 0x04 //Not receiving response from LTC6810 (cause safe state because we can't measure cell voltage and temperature anymore)
};

enum RELAY_STATE {
	OPEN,
	CLOSED
};

enum BATTERY_STATE {
	HEALTHY,
	FAULTED
};

enum DATA_RECEIVED {
	NOT_RECEIVED,
	RECEIVED
};

#endif
