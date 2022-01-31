/*
 * disp_renderer.h
 *
 *  Created on: Oct 5, 2019
 *      Author: jamesliu
 */

#ifndef DISP_RENDERER_H_
#define DISP_RENDERER_H_

#include "main.h"
#include "cmsis_os.h"
#include "SSD1322.h"
#include "lvgl/lvgl.h"

void displayTask(void* pv);
void displayInit();
void displayTmr(void* pv);
void disp_setMCMBPulseFreq(uint32_t hz); // critical
void disp_setMCMBSpeedUnit(uint8_t pi);
void disp_setMCMBDispState(uint32_t x);
void disp_setMCMBPwrEco(uint8_t pwr);
void disp_setMCMBFwdRev(uint8_t fwd);
void disp_setBBMBBusVoltage(uint32_t mv); // critical
void disp_setBBMBBusCurrent(uint32_t ma); // critical
void disp_setBBMBBmsAlertType(uint8_t type, uint32_t val); // critical
void disp_setPPTMBBusCurrent(uint32_t ma);
void disp_setDCMBLeftLightState(uint32_t on);
void disp_setDCMBRightLightState(uint32_t on);
void disp_setDCMBStopLightState(uint32_t on);
void disp_setDCMBHazardLightState(uint32_t on);
void disp_setDCMBIgnitionState(uint32_t on);
void disp_setDCMBArrayIgnitionState(uint32_t on);
void disp_setDCMBMotIgnitionState(uint32_t on);
void disp_setDCMBAccPotPosition(uint8_t x);
void disp_setCHASETargetSpeed(uint32_t kph);
void disp_setCHASEAlertType(uint32_t type);
void disp_setCHASETextMessage(uint8_t* pc, uint8_t len);
void disp_setCHASERealTime(uint64_t time);
void disp_attachMtaCallback(void(*cb)(uint8_t mta));
void disp_attachDriverAckCallback(void(*cb)(uint8_t x));
void disp_attachVfmUpCallback(void(*cb)(void));
void disp_attachVfmDownCallback(void(*cb)(void));
void disp_attachVfmResetCallback(void(*cb)(void));
void disp_attachAccResetCallback(void(*cb)(void));
void disp_attachRegenResetCallback(void(*cb)(void));
void disp_updateNavState(uint8_t up, uint8_t down, uint8_t left, uint8_t right, uint8_t sel, int16_t enc);
void disp_attachMotOnCallback(void(*cb)(uint8_t on));

enum{
	DISP_BMS_ALERT_RESET = 0b00000000,
	DISP_BMS_ALERT_BUS_OV = 0b00000001,
	DISP_BMS_ALERT_BUS_UV = 0b00000010,
	DISP_BMS_ALERT_BUS_OC = 0b00000011,
	DISP_BMS_ALERT_CELL_OV = 0b00100000,
	DISP_BMS_ALERT_CELL_UV = 0b01000000,
	DISP_BMS_ALERT_CELL_OC = 0b01100000,
	DISP_BMS_ALERT_CELL_OT = 0b10000000,
	DISP_BMS_ALERT_CELL_UT = 0b10100000,
};

#endif /* DISP_RENDERER_H_ */
