/*
 * SSD1322.h
 *
 *  Created on: Oct 5, 2019
 *      Author: jamesliu
 */

#ifndef SSD1322_H_
#define SSD1322_H_

#include "main.h"
#include "cmsis_os.h"
#include "lvgl/lvgl.h"

void SSD_init();
void SSD_init_hack();
void SSD_writeRegion(lv_area_t * area, uint8_t* buf);
void SSD_writeRegion_hack(lv_area_t * area, uint8_t* buf);
void SSD_clearDisplay();
void SSD_test();
void my_rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area);
void my_disp_flush(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p);
void my_disp_flush_hack(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p);
bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data);

#endif /* SSD1322_H_ */
