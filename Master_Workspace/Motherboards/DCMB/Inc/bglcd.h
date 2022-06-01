/*
 * bglcd.h
 *
 *  Created on: May 28, 2022
 *      Author: tristan
 */

#ifndef BGLCD_H_
#define BGLCD_H_

void drawP1Default(int value[4]);
void drawP1Detailed(int value[9]);
void drawP1Activate();
void drawP1Deactivate();
void drawP1IgnitionOff();
void drawP1BMSFault();
void drawP1(uint8_t sel);
void drawP2Default(int value[4]);
void drawP2Detailed(int value[4]);
void drawP2Activate();
void drawP2Deactivate();
void drawP2IgnitionOff(int value[4]);
void drawP2BMSFault(int value[4]);
void drawP2(uint8_t sel);
#endif /* BGLCD_H_ */
