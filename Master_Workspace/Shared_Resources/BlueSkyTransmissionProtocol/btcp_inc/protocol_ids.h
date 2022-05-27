

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
	BBMB_CELL_METRICS_ID,
	BBMB_BSD_ID,
	BBMB_BMS_MCU_STATUS_ID,
	BBMB_STATE_OF_CHARGE_ID,
	BBMB_SOC_REQUEST = 0x05,
	
	BBMB_LP_BUS_METRICS_ID = 0x0D,
	BBMB_CORE_TEMP_ID = 0x0E,
	BBMB_HEARTBEAT_ID = 0x0F
};

enum PPTMB_DATA_ID {
	PPTMB_BUS_METRICS_ID,
	PPTMB_PPT_METRICS_ID,
	
	PPTMB_LP_BUS_METRICS_ID = 0x0D,
	PPTMB_CORE_TEMP_ID = 0x0E,
	PPTMB_HEARTBEAT_ID = 0x0F
};

enum MCMB_Data_ID {
	MCMB_BUS_METRICS_ID,
	MCMB_SPEED_PULSE_ID,
	MCMB_MOTOR_TEMPERATURE_ID,
	
	MCMB_LP_BUS_METRICS_ID = 0x0D,
	MCMB_CORE_TEMP_ID = 0x0E,
	MCMB_HEARTBEAT_ID = 0x0F
};

enum DCMB_Data_ID {
	DCMB_MC2_STATE_ID,
	DCMB_BBOX_STARTUP_ID,
	DCMB_PPTBOX_STARTUP_ID,
	DCMB_LIGHTCONTROL_ID,
	DCMB_HORNSTATE_ID,
	
	DCMB_LP_BUS_METRICS_ID = 0x0D,
	DCMB_CORE_TEMP_ID = 0x0E,
	DCMB_HEARTBEAT_ID = 0x0F
};

enum Chase_Data_ID {
	CHASE_HEARTBEAT_REQ_ID,
	CHASE_SOFT_RESET_REQ_ID,
	CHASE_SET_FREQUENCY_ID,
	CHASE_SET_RTC_ID
};

enum BMS_Data_ID {
	BMS_CELL_TEMP,
	BMS_CELL_VOLT,
	BMS_CELL_SO = 0x04
};


#endif
