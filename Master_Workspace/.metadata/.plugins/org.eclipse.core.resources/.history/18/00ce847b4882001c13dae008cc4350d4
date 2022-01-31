/*
 * disp_renderer.c
 *
 *  Created on: Oct 5, 2019
 *      Author: jamesliu
 */

#include "disp_renderer.h"

//########     ###    ########     ###    ##     ##
//##     ##   ## ##   ##     ##   ## ##   ###   ###
//##     ##  ##   ##  ##     ##  ##   ##  #### ####
//########  ##     ## ########  ##     ## ## ### ##
//##        ######### ##   ##   ######### ##     ##
//##        ##     ## ##    ##  ##     ## ##     ##
//##        ##     ## ##     ## ##     ## ##     ##

static const float WHEEL_DIA_M = 0.559f;
static const float WHEEL_CIRC_M = 1.756150293f;
static const char* BMS_ALERT_MSG_RESET = "No BMS Errors";
static const char* BMS_ALERT_MSG_BUS_OV = "BUS OVERVOLTAGE!!! @ %d.%dV";
static const char* BMS_ALERT_MSG_BUS_UV = "BUS UNDERVOLTAGE!!! @ %d.%dV";
static const char* BMS_ALERT_MSG_BUS_OC = "BUS OVERCURRENT!!! @ %d.%dA";
static const char* BMS_ALERT_MSG_CELL_OV = "CELL OVERVOLTAGE!!! #%d @ %d.%dV";
static const char* BMS_ALERT_MSG_CELL_UV = "CELL UNDERVOLTAGE!!! #%d @ %d.%dV";
static const char* BMS_ALERT_MSG_CELL_OC = "CELL OVERCURRENT!!! #%d @ %d.%dA";
static const char* BMS_ALERT_MSG_CELL_OT = "CELL OVERTEMPERATURE!!! #%d @ %d.%d°C";
static const char* BMS_ALERT_MSG_CELL_UT = "CELL UNDERTEMPERATURE!!! #%d @ %d.%d°C";
static const char* BBMB_BUS_PWR_MSG = "%s %3d.%02d#999999 V, %c#%2d.%02d#999999 A, %c#%5d#999999 W OUT";
static const char* PPTMB_BUS_PWR_MSG = "%s %3d.%02d#999999 V, %c#%2d.%02d#999999 A, %c#%5d#999999 W OUT";
LV_IMG_DECLARE(DISP_main_bg);
LV_IMG_DECLARE(DISP_mot_bg)
LV_IMG_DECLARE(DISP_left_arrow);
LV_IMG_DECLARE(DISP_right_arrow);
LV_IMG_DECLARE(DISP_stop_sign);
LV_IMG_DECLARE(DISP_triangle_sign);
LV_IMG_DECLARE(DISP_Mot_Pwr_img);
LV_IMG_DECLARE(DISP_Mot_Eco_img);
LV_IMG_DECLARE(DISP_Mot_Fwd_img);
LV_IMG_DECLARE(DISP_Mot_Rev_img);
LV_IMG_DECLARE(DISP_Mot_On_img);
LV_IMG_DECLARE(DISP_Mot_Off_img);
LV_IMG_DECLARE(DISP_Text_bg);
LV_IMG_DECLARE(DISP_bubble_img);
extern const uint8_t kaboom[8192];

//########  ######## ########
//##     ## ##       ##     ##
//##     ## ##       ##     ##
//########  ######   ########
//##        ##       ##
//##        ##       ##
//##        ##       ##

static void lvglTick(void* id);
static void testTask(void* pv);
static void initStyles();
static void createObjects();
static void showMainPage(uint8_t en);
static void showMotPage(uint8_t en);
static void showTextPage(uint8_t en);
static void updateMotSpeed();
static void updateBattPwr();
static void updatePptPwr();
static void updateGearLabel();
static void updateVfm();
static void updateMta();
static void updateAcc();
static void updateRegen();
static void updateMotLcd();
static void updateMotOnBox();
static void notifTmr(TimerHandle_t xTimer);

// ######  ##     ##
//##    ## ##     ##
//##       ##     ##
// ######  ##     ##
//      ##  ##   ##
//##    ##   ## ##
// ######     ###

static SemaphoreHandle_t dispMtx;
static TimerHandle_t notifTmrHandle;
static uint32_t battV = 0;
static uint32_t battA = 0;
static uint32_t arrayV = 0;
static uint32_t arrayA = 0;
static uint32_t motV = 0;
static uint32_t motA = 0;
static struct state{
	int8_t encAcc;
	uint8_t pi :1;
	uint8_t page :1; //0=main, 1=mot
	uint8_t timerMode :1;
	uint8_t motOnWindow :1;
	uint8_t lastUp :1;
	uint8_t lastDown :1;
	uint8_t lastLeft :1;
	uint8_t lastRight :1;
	uint8_t lastSel :1;
	uint8_t mta :4;
	uint8_t vfm :4;
	uint8_t battOn :1;
	uint8_t arrOn :1;
	uint8_t motOn :1;
	uint16_t speedPulse;
	uint8_t acc;
	uint8_t regen;
	uint8_t fwd :1;
	uint8_t leftOn :1;
	uint8_t rightOn :1;
	uint8_t stopOn :1;
	uint8_t hazardOn :1;
	uint8_t motLedOn :1;
	uint8_t eco :1;
	uint8_t msg :1;
}state = {0};
void(*mtaCallback)(uint8_t mta);
void(*driverAckCallback)(uint8_t x);
void(*vfmUpCallback)(void);
void(*vfmDownCallback)(void);
void(*vfmResetCallback)(void);
void(*accResetCallback)(void);
void(*regenResetCallback)(void);
void(*motOnCallback)(uint8_t on);

// ######  ######## ##    ## ##       ########  ######
//##    ##    ##     ##  ##  ##       ##       ##    ##
//##          ##      ####   ##       ##       ##
// ######     ##       ##    ##       ######    ######
//      ##    ##       ##    ##       ##             ##
//##    ##    ##       ##    ##       ##       ##    ##
// ######     ##       ##    ######## ########  ######

static lv_style_t screenStl; // Screen Background
static lv_style_t barStl; // Indicator Bar Style
static lv_style_t bigNumStl; // Big Number
static lv_style_t targetSpeedStl; // Target Speed
static lv_style_t bigUnitStl; // Big Number's Units
static lv_style_t scrDivStl; // Screen Divider
static lv_style_t smlTxtStl; // Small Text
static lv_style_t pwrTxtStl; // Main Power Style
static lv_style_t gearTxtStl; // Gear Text Style
static lv_style_t mainVfmStl; // Main screen VFM Style
static lv_style_t motOnStl; // Mot On Box Style

static lv_style_t motVfmStl;
static lv_style_t motLcdStl;
static lv_style_t motMtaStl;
static lv_style_t motAccArcStl;
static lv_style_t motAccTxtStl;
static lv_style_t motLedStl;

static lv_style_t textMsgStl;

static void initStyles(){
	// Screen Background
	lv_style_copy(&screenStl, &lv_style_transp_tight);
	screenStl.body.main_color = lv_color_hex3(0x000);
	screenStl.text.color = lv_color_hex3(0x0F0);
	screenStl.text.font = &Hack_8_2FA1F;
	// Indicator Bar Style
	lv_style_copy(&barStl, &lv_style_plain);
	barStl.body.main_color = lv_color_hex3(0xaaa);
	barStl.body.opa = 255;
	// Big Number
	lv_style_copy(&bigNumStl, &lv_style_transp_tight);
	bigNumStl.body.main_color = lv_color_hex3(0x000);
	bigNumStl.text.color = lv_color_hex3(0x0F0);
	bigNumStl.text.font = &Hack_32_2FA1F;
	// Target Speed
	lv_style_copy(&targetSpeedStl, &bigNumStl);
	targetSpeedStl.text.font = &Hack_16_2FA1F;
	// Big Number's Units
	lv_style_copy(&bigUnitStl, &lv_style_transp_tight);
	bigUnitStl.body.main_color = lv_color_hex3(0x000);
	bigUnitStl.text.color = lv_color_hex3(0x090);
	bigUnitStl.text.font = &Hack_16_2FA1F;
	// Screen Divider
	lv_style_copy(&scrDivStl, &lv_style_transp_tight);
	scrDivStl.body.main_color = lv_color_hex3(0x0C0);
	scrDivStl.body.border.width = 1;
	scrDivStl.body.border.color = lv_color_hex3(0x0C0);
	// Small Text
	lv_style_copy(&smlTxtStl, &bigNumStl);
	smlTxtStl.text.font = &Hack_8_2FA1F;
	// Main Power Style
	lv_style_copy(&pwrTxtStl, &bigNumStl);
	pwrTxtStl.text.font = &Hack_12_2FA1F;
	// Gear Text Style
	lv_style_copy(&gearTxtStl, &bigNumStl);
	gearTxtStl.text.font = &Hack_24_2FA1F;
	gearTxtStl.text.color = lv_color_hex3(0x0C0);
	// Main screen VFM Style
	lv_style_copy(&mainVfmStl, &bigNumStl);
	mainVfmStl.text.font = &DISP_Ds_24_7F;
	mainVfmStl.text.color = lv_color_hex3(0xCCC);
	// Mot On Box Style
	lv_style_copy(&motOnStl, &lv_style_plain);
	motOnStl.text.font = &Hack_16_2FA1F;
	motOnStl.text.color = lv_color_hex3(0xFFF);
	motOnStl.body.border.width = 2;
	motOnStl.body.border.color = lv_color_hex3(0xFFF);
	motOnStl.body.opa = 255;
	motOnStl.body.border.opa = 255;
	motOnStl.body.main_color = lv_color_hex3(0x000);
	motOnStl.body.grad_color = lv_color_hex3(0x000);
	motOnStl.body.padding.inner = 7;

	// motVfmStl
	lv_style_copy(&motVfmStl, &bigNumStl);
	motVfmStl.text.font = &DISP_Ds_55_7F;
	motVfmStl.text.color = lv_color_hex3(0xCCC);
	motLcdStl.text.letter_space = 1;
	// motLcdStl
	lv_style_copy(&motLcdStl, &bigNumStl);
	motLcdStl.text.font = &DISP_Thumb_8_7F;
	motLcdStl.text.color = lv_color_hex3(0xFFF);
	motLcdStl.text.line_space = 0;
	// motMtaStl
	lv_style_copy(&motMtaStl, &bigNumStl);
	motMtaStl.text.font = &Hack_16_2FA1F;
	motMtaStl.text.color = lv_color_hex3(0xFFF);
	// motAccArcStl
	lv_style_copy(&motAccArcStl, &lv_style_plain);
	motAccArcStl.line.color = lv_color_hex3(0xFFF);
	motAccArcStl.line.width = 2;
	motAccArcStl.line.opa = 255;
	motOnStl.body.opa = 255;
	// motAccTxtStl
	lv_style_copy(&motAccTxtStl, &bigNumStl);
	motAccTxtStl.text.font = &Hack_12_2FA1F;
	motAccTxtStl.text.color = lv_color_hex3(0xFFF);
	// motLedStl
	lv_style_copy(&motLedStl, &barStl);
	motLedStl.body.radius = LV_RADIUS_CIRCLE;

	// textMsgStl
	lv_style_copy(&textMsgStl, &bigNumStl);
	textMsgStl.text.font = &Hack_12_2FA1F;
	textMsgStl.text.color = lv_color_hex3(0xFFF);
}

// #######  ########        ## ########  ######  ########  ######
//##     ## ##     ##       ## ##       ##    ##    ##    ##    ##
//##     ## ##     ##       ## ##       ##          ##    ##
//##     ## ########        ## ######   ##          ##     ######
//##     ## ##     ## ##    ## ##       ##          ##          ##
//##     ## ##     ## ##    ## ##       ##    ##    ##    ##    ##
// #######  ########   ######  ########  ######     ##     ######

static lv_obj_t* backgroundImg = NULL;
static lv_obj_t* bigSpeedLabel = NULL;
static lv_obj_t* bigUnitLabel = NULL;
static lv_obj_t* targetSpeedLabel = NULL;
static lv_obj_t* battPwrLabel = NULL;
static lv_obj_t* arrayPwrLabel = NULL;
static lv_obj_t* bmsAlertMessageLabel = NULL;
static lv_obj_t* accPositionObj = NULL;
static lv_obj_t* gearTxtLabel = NULL;
static lv_obj_t* mainVfmLabel = NULL;
static lv_obj_t* motOnLabel = NULL;
static lv_obj_t* motOnLabelText = NULL;
static lv_obj_t* leftArrowImg = NULL;
static lv_obj_t* rightArrowImg = NULL;
static lv_obj_t* stopSignImg = NULL;
static lv_obj_t* triangleSignImg = NULL;
static lv_obj_t* mainPageLedBar[11];
static lv_obj_t** mainPageObjs[] = {&backgroundImg,&bigSpeedLabel,&bigUnitLabel,&targetSpeedLabel,&battPwrLabel,&arrayPwrLabel,&bmsAlertMessageLabel,&accPositionObj,&gearTxtLabel,&mainVfmLabel,&motOnLabel,&leftArrowImg,&rightArrowImg,&stopSignImg,&triangleSignImg};

static lv_obj_t* motBgImg = NULL;
static lv_obj_t* motVfmLabel = NULL;
static lv_obj_t* motLcdLabel = NULL;
static lv_obj_t* motMtaLabel = NULL;
static lv_obj_t* motFwdImg = NULL;
static lv_obj_t* motRevImg = NULL;
static lv_obj_t* motPwrImg = NULL;
static lv_obj_t* motEcoImg = NULL;
static lv_obj_t* motOnImg = NULL;
static lv_obj_t* motOffImg = NULL;
static lv_obj_t* motAccArc = NULL;
static lv_obj_t* motRegenArc = NULL;
static lv_obj_t* motAccLabel = NULL;
static lv_obj_t* motRegenLabel = NULL;
static lv_obj_t* motLedCirc = NULL;
static lv_obj_t* motPageLedBar[11];
static lv_obj_t** motPageObjs[] = {&motBgImg,&motVfmLabel,&motLcdLabel,&motMtaLabel,&motFwdImg,&motRevImg,&motPwrImg,&motEcoImg,&motOnImg,&motOffImg,&motAccArc,&motRegenArc,&motAccLabel,&motRegenLabel,&motLedCirc};

static lv_obj_t* textBgImg = NULL;
static lv_obj_t* textMsgLabel = NULL;
static lv_obj_t* textNotifImg = NULL;

static void createObjects(){
	lv_obj_set_style(lv_scr_act(), &screenStl);

	// Main Screen
	backgroundImg = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(backgroundImg, &DISP_main_bg);
	lv_obj_set_pos(backgroundImg, 0, 0);

	bigSpeedLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(bigSpeedLabel, "  0");
	lv_label_set_style(bigSpeedLabel, LV_LABEL_STYLE_MAIN, &bigNumStl);
	lv_label_set_align(bigSpeedLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(bigSpeedLabel, -4, 6);

	bigUnitLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(bigUnitLabel, "kmph");
	lv_label_set_style(bigUnitLabel, LV_LABEL_STYLE_MAIN, &bigUnitStl);
	lv_label_set_align(bigUnitLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(bigUnitLabel, 54, 22);

	targetSpeedLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(targetSpeedLabel, "000");
	lv_label_set_style(targetSpeedLabel, LV_LABEL_STYLE_MAIN, &targetSpeedStl);
	lv_label_set_align(targetSpeedLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(targetSpeedLabel, 73, -3);

	battPwrLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(battPwrLabel, "○ 0.00#999999 V, +# 0.00#999999 A, -#    0#999999 W OUT");
	lv_label_set_recolor(battPwrLabel, true);
	lv_label_set_style(battPwrLabel, LV_LABEL_STYLE_MAIN, &pwrTxtStl);
	lv_label_set_align(battPwrLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(battPwrLabel, 37, 40);

	arrayPwrLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(arrayPwrLabel, "○ 0.00#999999 V, +# 0.00#999999 A, +#    0#999999 W OUT");
	lv_label_set_recolor(arrayPwrLabel, true);
	lv_label_set_style(arrayPwrLabel, LV_LABEL_STYLE_MAIN, &pwrTxtStl);
	lv_label_set_align(arrayPwrLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(arrayPwrLabel, 37, 51);

	bmsAlertMessageLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(bmsAlertMessageLabel, BMS_ALERT_MSG_RESET);
	lv_label_set_style(bmsAlertMessageLabel, LV_LABEL_STYLE_MAIN, &smlTxtStl);
	lv_label_set_align(bmsAlertMessageLabel, LV_LABEL_ALIGN_LEFT);
	lv_label_set_long_mode(bmsAlertMessageLabel, LV_LABEL_LONG_SROLL_CIRC);
	lv_obj_set_pos(bmsAlertMessageLabel, 133, 30);
	lv_obj_set_width(bmsAlertMessageLabel, 122);

	gearTxtLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(gearTxtLabel, "P");
	lv_label_set_style(gearTxtLabel, LV_LABEL_STYLE_MAIN, &gearTxtStl);
	lv_label_set_align(gearTxtLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(gearTxtLabel, 132, 0);

	mainVfmLabel = lv_label_create(backgroundImg, NULL);
	lv_label_set_text(mainVfmLabel, "01");
	lv_label_set_style(mainVfmLabel, LV_LABEL_STYLE_MAIN, &mainVfmStl);
	lv_label_set_align(mainVfmLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(mainVfmLabel, 220, 9);

	leftArrowImg = lv_img_create(backgroundImg, NULL);
	lv_img_set_src(leftArrowImg, &DISP_left_arrow);
	lv_obj_set_pos(leftArrowImg, 149, 6);
	lv_obj_set_hidden(leftArrowImg, 1);

	rightArrowImg = lv_img_create(backgroundImg, NULL);
	lv_img_set_src(rightArrowImg, &DISP_right_arrow);
	lv_obj_set_pos(rightArrowImg, 191, 6);
	lv_obj_set_hidden(rightArrowImg, 1);

	stopSignImg = lv_img_create(backgroundImg, NULL);
	lv_img_set_src(stopSignImg, &DISP_stop_sign);
	lv_obj_set_pos(stopSignImg, 111, 13);
	lv_obj_set_hidden(stopSignImg, 1);

	triangleSignImg = lv_img_create(backgroundImg, NULL);
	lv_img_set_src(triangleSignImg, &DISP_triangle_sign);
	lv_obj_set_pos(triangleSignImg, 171, 6);
	lv_obj_set_hidden(triangleSignImg, 1);

	accPositionObj = lv_obj_create(backgroundImg, NULL);
	lv_obj_set_style(accPositionObj, &barStl);
	lv_obj_set_pos(accPositionObj, 56, 18);
	lv_obj_set_width(accPositionObj, 0);
	lv_obj_set_height(accPositionObj, 2);

	motOnLabel = lv_obj_create(backgroundImg, NULL);
	lv_obj_set_style(motOnLabel, &motOnStl);
	lv_obj_set_pos(motOnLabel, 63, 8);
	lv_obj_set_width(motOnLabel, 130);
	lv_obj_set_height(motOnLabel, 48);
	lv_obj_set_hidden(motOnLabel, 1);

	motOnLabelText = lv_label_create(motOnLabel, NULL);
	lv_label_set_text(motOnLabelText, "Press SEL to\ntoggle motor");
	lv_label_set_style(motOnLabelText, LV_LABEL_STYLE_MAIN, &motOnStl);
	lv_obj_set_style(motOnLabelText, &motOnStl);
	lv_label_set_align(motOnLabelText, LV_LABEL_ALIGN_CENTER);
	lv_obj_set_pos(motOnLabelText, 5, 2);
	lv_obj_set_width(motOnLabelText, 130);
	lv_obj_set_height(motOnLabelText, 48);

	// Motor Screen
	motBgImg = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(motBgImg, &DISP_mot_bg);
	lv_obj_set_pos(motBgImg, 0, 0);

	motVfmLabel = lv_label_create(motBgImg, NULL);
	lv_label_set_text(motVfmLabel, "01");
	lv_label_set_style(motVfmLabel, LV_LABEL_STYLE_MAIN, &motVfmStl);
	lv_label_set_align(motVfmLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(motVfmLabel, -3, 10);

	motLcdLabel = lv_label_create(motBgImg, NULL);
	lv_label_set_text(motLcdLabel, "Lorem Ipsum Dolor Lo\nrem Ipsum Dolor Lore\nm Ipsum Dolor Lorem \nIpsum Dolor Lorem Ip");
	lv_label_set_style(motLcdLabel, LV_LABEL_STYLE_MAIN, &motLcdStl);
	lv_label_set_align(motLcdLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(motLcdLabel, 74, 19);

	motMtaLabel = lv_label_create(motBgImg, NULL);
	lv_label_set_text(motMtaLabel, "00/15");
	lv_label_set_style(motMtaLabel, LV_LABEL_STYLE_MAIN, &motMtaStl);
	lv_label_set_align(motMtaLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(motMtaLabel, 101, 48);

	motAccLabel = lv_label_create(motBgImg, NULL);
	lv_label_set_text(motAccLabel, "000");
	lv_label_set_style(motAccLabel, LV_LABEL_STYLE_MAIN, &motAccTxtStl);
	lv_label_set_align(motAccLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(motAccLabel, 232, 6);

	motRegenLabel = lv_label_create(motBgImg, NULL);
	lv_label_set_text(motRegenLabel, "000");
	lv_label_set_style(motRegenLabel, LV_LABEL_STYLE_MAIN, &motAccTxtStl);
	lv_label_set_align(motRegenLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(motRegenLabel, 232, 39);

	motFwdImg = lv_img_create(motBgImg, NULL);
	lv_img_set_src(motFwdImg, &DISP_Mot_Fwd_img);
	lv_obj_set_pos(motFwdImg, 167, 18);
	lv_obj_set_hidden(motFwdImg, 1);

	motRevImg = lv_img_create(motBgImg, NULL);
	lv_img_set_src(motRevImg, &DISP_Mot_Rev_img);
	lv_obj_set_pos(motRevImg, 192, 18);
	lv_obj_set_hidden(motRevImg, 1);

	motPwrImg = lv_img_create(motBgImg, NULL);
	lv_img_set_src(motPwrImg, &DISP_Mot_Pwr_img);
	lv_obj_set_pos(motPwrImg, 167, 34);
	lv_obj_set_hidden(motPwrImg, 1);

	motEcoImg = lv_img_create(motBgImg, NULL);
	lv_img_set_src(motEcoImg, &DISP_Mot_Eco_img);
	lv_obj_set_pos(motEcoImg, 192, 34);
	lv_obj_set_hidden(motEcoImg, 1);

	motOnImg = lv_img_create(motBgImg, NULL);
	lv_img_set_src(motOnImg, &DISP_Mot_On_img);
	lv_obj_set_pos(motOnImg, 167, 50);
	lv_obj_set_hidden(motOnImg, 1);

	motOffImg = lv_img_create(motBgImg, NULL);
	lv_img_set_src(motOffImg, &DISP_Mot_Off_img);
	lv_obj_set_pos(motOffImg, 192, 50);
	lv_obj_set_hidden(motOffImg, 1);

	motAccArc = lv_arc_create(motBgImg, NULL);
	lv_arc_set_style(motAccArc, LV_ARC_STYLE_MAIN, &motAccArcStl);
	lv_arc_set_angles(motAccArc, 315, 315);
	lv_obj_set_size(motAccArc, 28, 28);
	lv_obj_set_pos(motAccArc, 228, 0);

	motRegenArc = lv_arc_create(motBgImg, NULL);
	lv_arc_set_style(motRegenArc, LV_ARC_STYLE_MAIN, &motAccArcStl);
	lv_obj_set_size(motRegenArc, 28, 28);
	lv_obj_set_pos(motRegenArc, 228, 33);
	lv_arc_set_angles(motRegenArc, 315, 315);

	motLedCirc = lv_obj_create(motBgImg, NULL);
	lv_arc_set_style(motLedCirc, LV_ARC_STYLE_MAIN, &motLedStl);
	lv_arc_set_angles(motLedCirc, 0, 360);
	lv_obj_set_pos(motLedCirc, 192, 2);
	lv_obj_set_size(motLedCirc, 12, 12);

	textBgImg = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(textBgImg, &DISP_Text_bg);
	lv_obj_set_pos(textBgImg, 0, 0);
	lv_obj_set_hidden(textBgImg, 1);

	textMsgLabel = lv_label_create(textBgImg, NULL);
	lv_label_set_text(textMsgLabel, "000");
	lv_label_set_style(textMsgLabel, LV_LABEL_STYLE_MAIN, &textMsgStl);
	lv_label_set_align(textMsgLabel, LV_LABEL_ALIGN_LEFT);
	lv_obj_set_pos(textMsgLabel, 0, -2);
	lv_obj_set_size(textMsgLabel, 256, 66);

	textNotifImg = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(textNotifImg, &DISP_bubble_img);
	lv_obj_set_pos(textNotifImg, -1, -1);
	lv_obj_set_hidden(textNotifImg, 1);
}

//######## ##     ## ##    ##  ######
//##       ##     ## ###   ## ##    ##
//##       ##     ## ####  ## ##
//######   ##     ## ## ## ## ##
//##       ##     ## ##  #### ##
//##       ##     ## ##   ### ##    ##
//##        #######  ##    ##  ######

static void showMainPage(uint8_t en){
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	if(en){
		lv_obj_set_hidden(leftArrowImg, !state.leftOn);
		lv_obj_set_hidden(rightArrowImg, !state.rightOn);
		lv_obj_set_hidden(stopSignImg, !state.stopOn);
		lv_obj_set_hidden(triangleSignImg, !state.hazardOn);
		lv_obj_set_hidden(motOnLabel,!state.motOnWindow);
	}
	lv_obj_set_hidden(backgroundImg,!en);
	xSemaphoreGive(dispMtx);
}

static void showMotPage(uint8_t en){
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	if(en){
		lv_obj_set_hidden(motFwdImg, !state.fwd);
		lv_obj_set_hidden(motRevImg, state.fwd);
		lv_obj_set_hidden(motPwrImg, state.eco);
		lv_obj_set_hidden(motEcoImg, !state.eco);
		lv_obj_set_hidden(motOnImg, !state.motOn);
		lv_obj_set_hidden(motOffImg, state.motOn);
		lv_obj_set_hidden(motLedCirc, !state.motLedOn);
	}
	lv_obj_set_hidden(motBgImg,!en);
	xSemaphoreGive(dispMtx);
}

static void showTextPage(uint8_t en){
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(textBgImg, !en);
	if(en){
		lv_obj_set_hidden(textNotifImg, 1);
		state.msg = 0;
	}
	xSemaphoreGive(dispMtx);
}

static void testTask(void* pv){
	osDelay(1000);
	for(int i=0; i<40; i++){
		disp_setMCMBPulseFreq(i);
		osDelay(50);
	}
	for(int i=0; i<40; i++){
		disp_setBBMBBusVoltage(i*3000);
		osDelay(50);
	}
	for(int i=0; i<40; i++){
		disp_setBBMBBusCurrent(i*250);
		osDelay(50);
	}
	for(int i=0; i<40; i++){
		disp_setBBMBBmsAlertType(DISP_BMS_ALERT_BUS_UV, i*1000);
		osDelay(50);
	}
	for(int i=0; i<4; i++){
		disp_setDCMBLeftLightState(!(i&1));
		osDelay(500);
	}
	for(int i=0; i<4; i++){
		disp_setDCMBRightLightState(!(i&1));
		osDelay(500);
	}
	for(int i=0; i<4; i++){
		disp_setDCMBStopLightState(!(i&1));
		osDelay(500);
	}
	for(int i=0; i<4; i++){
		disp_setDCMBHazardLightState(!(i&1));
		osDelay(500);
	}
	vTaskDelete(NULL);
}

void displayInit(){
	dispMtx = xSemaphoreCreateMutex();
//	xSemaphoreTake(dispMtx, portMAX_DELAY);
//	osDelay(50);
	SSD_init_hack();
	lv_init();
	initStyles();
	xTimerStart(xTimerCreate("", 1, pdTRUE, NULL, lvglTick),0);
	notifTmrHandle = xTimerCreate("", 500, pdTRUE, NULL, notifTmr);
	lv_disp_buf_t* disp_buf = pvPortMalloc(sizeof(lv_disp_buf_t));
	lv_color_t* buf = pvPortMalloc((LV_HOR_RES_MAX * LV_VER_RES_MAX) * sizeof(lv_color_t));
	lv_disp_buf_init(disp_buf, buf, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	lv_disp_drv_t* disp_drv = pvPortMalloc(sizeof(lv_disp_drv_t));
	lv_disp_drv_init(disp_drv);
	disp_drv->flush_cb = my_disp_flush_hack;
	disp_drv->buffer = disp_buf;
	disp_drv->rounder_cb = my_rounder_cb;
	lv_disp_drv_register(disp_drv);
	createObjects();
	showMotPage(0);
	state.vfm = 1;
	state.fwd = 1;
	state.msg = 0;
	state.pi = 0;
//	xSemaphoreGive(dispMtx);
}

void displayInitTmr(void* pv){
}

void displayTmr(TimerHandle_t xTimer){
	uint32_t id = pvTimerGetTimerID(xTimer);
	if(id == 0){
		vTimerSetTimerID(xTimer, 1);
	}else if(id == 1){
		lv_area_t area;
		area.x1 = 0;
		area.x2 = 255;
		area.y1 = 0;
		area.y2 = 63;
		SSD_writeRegion_hack(&area, kaboom);
		xTimerChangePeriod(xTimer, 1975, portMAX_DELAY);
		vTimerSetTimerID(xTimer, 2);
	}else if(id == 2){
		xTimerChangePeriod(xTimer, 25, portMAX_DELAY);
		vTimerSetTimerID(xTimer, 3);
	}else{
		if(xSemaphoreTake(dispMtx, 0) == pdPASS){
			lv_task_handler();
			xSemaphoreGive(dispMtx);
		}
	}
}

static void notifTmr(TimerHandle_t xTimer){
	if(state.msg){
		lv_obj_set_hidden(textNotifImg, !lv_obj_get_hidden(textNotifImg));
	}else{
		lv_obj_set_hidden(textNotifImg, 1);
		xTimerStop(notifTmrHandle, 0);
	}
}

void displayTask(void* pv){
	dispMtx = xSemaphoreCreateMutex();
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	osDelay(50);
	SSD_init_hack();
	lv_init();
	initStyles();
	xTimerStart(xTimerCreate("", 1, pdTRUE, NULL, lvglTick),0);
	lv_disp_buf_t disp_buf;
	lv_color_t* buf = pvPortMalloc((LV_HOR_RES_MAX * LV_VER_RES_MAX) * sizeof(lv_color_t));
	lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.flush_cb = my_disp_flush_hack;
	disp_drv.buffer = &disp_buf;
	disp_drv.rounder_cb = my_rounder_cb;
	lv_disp_drv_register(&disp_drv);
	createObjects();
	xSemaphoreGive(dispMtx);
	//xTaskCreate(testTask, "", 256, NULL, 3, NULL);
	for(;;){
		xSemaphoreTake(dispMtx, portMAX_DELAY);
		lv_task_handler();
		xSemaphoreGive(dispMtx);
		osDelay(10);
	}
}

static void lvglTick(void* id){
	lv_tick_inc(1);
}

static void updateMotSpeed(){
	float speed = state.pi ? (float)state.speedPulse / 16.0f * WHEEL_DIA_M : (float)state.speedPulse / 16.0f * WHEEL_CIRC_M * 3.6f;
	uint8_t buf[10];
	sprintf(buf, "%3d", (int)speed);
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(bigSpeedLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updateBattPwr(){
	uint8_t buf[64];
	sprintf(buf, BBMB_BUS_PWR_MSG, state.motOn?"●":"○", battV/1000, battV%1000/10, battA<0?'-':'+', battA/1000, battA%1000/10, battA<0?'-':'+', (int)((float)battV*(float)battA/1e6));
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(battPwrLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updatePptPwr(){
	uint8_t buf[64];
	sprintf(buf, PPTMB_BUS_PWR_MSG, state.arrOn?"●":"○", arrayV/1000, arrayV%1000/10, arrayA<0?'-':'+', arrayA/1000, arrayA%1000/10, arrayA<0?'-':'+', (int)((float)arrayV*(float)arrayA/1e6));
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(arrayPwrLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updateGearLabel(){
	uint8_t buf[2] = {0};
	if(!state.motOn) buf[0] = 'P';
	else{
		if(state.fwd) buf[0] = 'D';
		else buf[0] = 'R';
	}
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(gearTxtLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updateVfm(){
	uint8_t buf[4];
	sprintf(buf, "%02d", state.vfm);
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(motVfmLabel, buf);
	lv_label_set_text(mainVfmLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updateMta(){
	uint8_t buf[8];
	sprintf(buf, "%02d/15", state.mta);
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(motMtaLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updateAcc(){
	uint8_t buf[8];
	sprintf(buf, "%03d", state.acc);
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_width(accPositionObj, (uint32_t)state.acc*43/255);
	lv_arc_set_angles(motAccArc, 315-(int)((float)state.acc*270.0f/255.0f), 315);
	lv_label_set_text(motAccLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updateRegen(){
	uint8_t buf[8];
	sprintf(buf, "%03d", state.regen);
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_arc_set_angles(motRegenArc, 315-(int)((float)state.regen*270.0f/255.0f), 315);
	lv_label_set_text(motRegenLabel, buf);
	xSemaphoreGive(dispMtx);
}

static void updateMotLcd(){

}

static void updateMotOnBox(){
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(motOnLabel, !state.motOnWindow);
	xSemaphoreGive(dispMtx);
}


//##     ##    ###    ##    ## ########  ##       ######## ########   ######
//##     ##   ## ##   ###   ## ##     ## ##       ##       ##     ## ##    ##
//##     ##  ##   ##  ####  ## ##     ## ##       ##       ##     ## ##
//######### ##     ## ## ## ## ##     ## ##       ######   ########   ######
//##     ## ######### ##  #### ##     ## ##       ##       ##   ##         ##
//##     ## ##     ## ##   ### ##     ## ##       ##       ##    ##  ##    ##
//##     ## ##     ## ##    ## ########  ######## ######## ##     ##  ######

void disp_setMCMBPulseFreq(uint32_t hz){ // critical
	state.speedPulse = hz;
	updateMotSpeed();
}

void disp_setMCMBSpeedUnit(uint8_t pi){
	state.pi = pi;
	updateMotSpeed();
}

void disp_setMCMBDispState(uint32_t x); // no capture

void disp_setMCMBPwrEco(uint8_t pwr){
	state.eco = !pwr;
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(motPwrImg, !pwr);
	lv_obj_set_hidden(motEcoImg, pwr);
	xSemaphoreGive(dispMtx);
}

void disp_setMCMBFwdRev(uint8_t fwd){
	state.fwd = fwd;
	updateGearLabel();
	xSemaphoreTake(dispMtx, portMAX_DELAY);
//	if(state.page == 1){
	lv_obj_set_hidden(motFwdImg, !fwd);
	lv_obj_set_hidden(motRevImg, fwd);
//	}
	xSemaphoreGive(dispMtx);
}

void disp_setMCMBMotLed(uint8_t on){
	state.motLedOn = on;
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(motLedCirc, !on);
	xSemaphoreGive(dispMtx);
}

void disp_setBBMBBusVoltage(uint32_t mv){ // critical
	battV = mv;
	updateBattPwr();
}

void disp_setBBMBBusCurrent(uint32_t ma){ // critical
	battA = ma;
	updateBattPwr();
}

void disp_setBBMBBmsAlertType(uint8_t type, uint32_t val){ // critical
	uint8_t buf[64];
	switch (type>>4){
	case 0b0000: // bus metric fault
		switch (type&0xf){
		case 0b0000: // reset
			sprintf(buf, BMS_ALERT_MSG_RESET);
			break;
		case 0b0001: // OV
			sprintf(buf, BMS_ALERT_MSG_BUS_OV, val/1000, val%1000);
			break;
		case 0b0010: // UV
			sprintf(buf, BMS_ALERT_MSG_BUS_UV, val/1000, val%1000);
			break;
		case 0b0011: // OC
			sprintf(buf, BMS_ALERT_MSG_BUS_OC, val/1000, val%1000);
			break;
		}
		break;
	case 0b0001: // OV
		sprintf(buf, BMS_ALERT_MSG_CELL_OV, type&0x1f, val/1000, val%1000);
		break;
	case 0b0010: // UV
		sprintf(buf, BMS_ALERT_MSG_CELL_UV, type&0x1f, val/1000, val%1000);
		break;
	case 0b0011: // OC
		sprintf(buf, BMS_ALERT_MSG_CELL_OC, type&0x1f, val/1000, val%1000);
		break;
	case 0b0100: // OT
		sprintf(buf, BMS_ALERT_MSG_CELL_OT, type&0x1f, val/1000, val%1000);
		break;
	case 0b0101: // UT
		sprintf(buf, BMS_ALERT_MSG_CELL_UT, type&0x1f, val/1000, val%1000);
		break;
	}
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(bmsAlertMessageLabel, buf);
	xSemaphoreGive(dispMtx);
}

void disp_setPPTMBBusVoltage(uint32_t mv){
	arrayV = mv;
	updatePptPwr();
}

void disp_setPPTMBBusCurrent(uint32_t ma){
	arrayA = ma;
	updatePptPwr();
}

void disp_setDCMBLeftLightState(uint32_t on){ // critical
	state.leftOn = on;
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(leftArrowImg, !on);
	xSemaphoreGive(dispMtx);
}

void disp_setDCMBRightLightState(uint32_t on){ // critical
	state.rightOn = on;
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(rightArrowImg, !on);
	xSemaphoreGive(dispMtx);
}

void disp_setDCMBStopLightState(uint32_t on){ // critical
	state.stopOn = on;
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(stopSignImg, !on);
	xSemaphoreGive(dispMtx);
}

void disp_setDCMBHazardLightState(uint32_t on){ // critical
	state.hazardOn = on;
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_obj_set_hidden(triangleSignImg, !on);
	xSemaphoreGive(dispMtx);
}

void disp_setDCMBIgnitionState(uint32_t on){
	state.battOn = on;
	updateBattPwr();
}

void disp_setDCMBArrayIgnitionState(uint32_t on){
	state.arrOn = on;
	updatePptPwr();
}
void disp_setDCMBMotIgnitionState(uint32_t on){
	state.motOn = on;
	//updateMotOnSw();
	updateGearLabel();
	if(!on){
		state.vfm = 1;
		updateVfm();
		if(vfmResetCallback) vfmResetCallback();
		state.mta = 0;
		updateMta();
		if(mtaCallback) mtaCallback(0);
		state.acc = 0;
		updateAcc();
		if(accResetCallback) accResetCallback();
		state.regen = 0;
		updateRegen();
		if(regenResetCallback) regenResetCallback();
		updateMotLcd();
	}
}

void disp_setDCMBAccPotPosition(uint8_t x){
	state.acc = x;
	updateAcc();
}

void disp_setCHASETargetSpeed(uint32_t kph){
	uint8_t buf[8];
	sprintf(buf, "%3d", kph);
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(targetSpeedLabel, buf);
	xSemaphoreGive(dispMtx);
}

void disp_setCHASEAlertType(uint32_t type);

void disp_setCHASETextMessage(uint8_t* pc, uint8_t len){
	if(len > 176) len = 176;
	pc[len-1] = '\0';
	xSemaphoreTake(dispMtx, portMAX_DELAY);
	lv_label_set_text(textMsgLabel, pc);
	state.msg = 1;
	lv_obj_set_hidden(textNotifImg, 0);
	if(len >= 7){
		if(pc[0]=='#'&&pc[1]=='C'&&pc[2]=='S'){
			lv_label_set_text(targetSpeedLabel, pc+3);
		}
	}
	xSemaphoreGive(dispMtx);
	xTimerStart(notifTmr, 0);
}

void disp_setCHASERealTime(uint64_t time);

void disp_attachMtaCallback(void(*cb)(uint8_t mta)){
	mtaCallback = cb;
}

void disp_attachDriverAckCallback(void(*cb)(uint8_t x)){
	driverAckCallback = cb;
}

void disp_attachVfmUpCallback(void(*cb)(void)){
	vfmUpCallback = cb;
}

void disp_attachVfmDownCallback(void(*cb)(void)){
	vfmDownCallback = cb;
}

void disp_attachVfmResetCallback(void(*cb)(void)){
	vfmResetCallback = cb;
}

void disp_attachAccResetCallback(void(*cb)(void)){
	accResetCallback = cb;
}

void disp_attachRegenResetCallback(void(*cb)(void)){
	regenResetCallback = cb;
}

void disp_attachMotOnCallback(void(*cb)(uint8_t on)){
	motOnCallback = cb;
}

void disp_updateNavState(uint8_t up, uint8_t down, uint8_t left, uint8_t right, uint8_t sel, int16_t enc){
	// edge detect
	int8_t upEdge = up-state.lastUp;
	int8_t downEdge = down-state.lastDown;
	int8_t leftEdge = left-state.lastLeft;
	int8_t rightEdge = right-state.lastRight;
	int8_t selEdge = sel-state.lastSel;
	// encoder accumulate
	state.encAcc += enc;
	// motor on logic
	if(state.motOnWindow == 1){
		if(selEdge == 1){
			state.motOn = state.motOn ? 0 : 1;
			state.motOnWindow = 0;
			updateMotOnBox();
			disp_setDCMBMotIgnitionState(state.motOn);
			if(motOnCallback) motOnCallback(state.motOn);
			downEdge = upEdge = leftEdge = rightEdge = selEdge = state.encAcc = 0;
		}else if(downEdge == 1 || upEdge == 1 || leftEdge == 1 || rightEdge == 1 || state.encAcc >= 2 || state.encAcc <= -2){
			state.motOnWindow = 0;
			updateMotOnBox();
			downEdge = upEdge = leftEdge = rightEdge = selEdge = state.encAcc = 0;
		}
	}else{
		if(state.page == 0){
			if(downEdge == 1){
				state.motOnWindow = 1;
				updateMotOnBox();
				downEdge = upEdge = leftEdge = rightEdge = selEdge = state.encAcc = 0;
			}
		}
		// page select
		if(state.encAcc >= 2){
			if(state.page < 2) state.page++;
			state.encAcc = 0;
			if(state.page == 1){
				showMainPage(0);
				showMotPage(1);
				showTextPage(0);
			}
			if(state.page == 2){
				showMainPage(0);
				showMotPage(0);
				showTextPage(1);
			}
		}else if(state.encAcc <= -2){
			if(state.page > 0) state.page--;
			state.encAcc = 0;
			if(state.page == 0){
				showMotPage(0);
				showMainPage(1);
				showTextPage(0);
			}
			if(state.page == 1){
				showMotPage(1);
				showMainPage(0);
				showTextPage(0);
			}
		}
		// mot page logic
		if(state.page == 1){
			if(downEdge == 1){
				if(state.vfm > 1){
					state.vfm--;
					updateVfm();
					if(vfmDownCallback) vfmDownCallback();
				}
			}
			if(upEdge == 1){
				if(state.vfm < 8){
					state.vfm++;
					updateVfm();
					if(vfmUpCallback) vfmUpCallback();
				}
			}
			if(leftEdge == 1){
				if(state.mta > 0){
					state.mta--;
					updateMta();
					if(mtaCallback) mtaCallback(state.mta);
				}
			}
			if(rightEdge == 1){
				if(state.mta < 15){
					state.mta++;
					updateMta();
					if(mtaCallback) mtaCallback(state.mta);
				}
			}
		}
	}
	state.lastUp = up;
	state.lastDown = down;
	state.lastLeft = left;
	state.lastRight = right;
}
