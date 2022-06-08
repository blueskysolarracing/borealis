#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"

//------ MACROS ------//
#define DEFAULT_PWM_FREQ 30
#define DEFAULT_PWM_DUTY_CYCLE 0.8
#define VDDA 3.543 //Measured voltage on VDDA pin of MCU (for ADC measurements)
#define I_SENSE_AMP_RATIO 114 //Amplification ratio of current sense amp
#define I_SHUNT_VALUE 0.002 //Resistance of the current shunt (in Ohms)
#define MPPT_ITERATION_DELAY 1000 //Delay between iteration through MPPT loop (ms)
#define DELAY_BT_MPPT 1000 //Delay between update of the MPPT loop (in ms)
#define ALLOWED_IncCond_dV_ERROR 0.05
#define ALLOWED_IncCond_dIdV_ERROR 0.05
#define ALLOWED_IncCond_dI_ERROR 0.02
#define ALLOWED_HillClimb_dP_error 0.1 //Tolerance around power measurements to determine whether the change duty cycle
#define PWM_DUTY_CYCLE_STEP_CHANGE 0.02
#define I_MEAS_OFFSET 0.335 //Voltage offset on ADC input of current sense signal (need to check dependency on frequency and voltages)
#define MIN_PWM_DUTY_CYCLE 0.02 //Maximum allowed PWM duty cycle
#define MAX_PWM_DUTY_CYCLE 0.99 //Maximum allowed PWM duty cycle
#define NUM_AVG_CURRENT 5 //Number of samples in current measurements averaging
#define MIN_INPUT_VOLTAGE 1.0 //Minimum allowed input voltage on uMPPT #1 and #3
#define NUM_UMPPT 2 //Number of uMPPT on the board (1 to 5). Assumes that uMPPT are populated from #1 upwards and unused uMPPT are jumped

#define APB1_Initial_clk_freq 32000 //32 MHz
#define APB2_Initial_clk_freq 32000 //32 MHz

#define DEBUG_FLAG 0 //Flag to print debug statements
#define ADC_DEBUG_FLAG 0 //Flag to print ADC debug statements

//------ GLOBAL VARIABLES ------//
struct uMPPT { //Struct to hold data for a single uMPPT on the board
	uint8_t pwm_num; //# ID of uMPPT/PWM
	double pwm_phase_offset; //Target phase offset of PWM signal signal sent to gate driver; 0 to 1
	double pwm_duty_cycle; //Target duty cycle of PWM signal sent to gate driver; 0 to 1
	double pwm_frequency; //Frequency of PWM signal sent to gate driver;
	double calc_input_current; //Calculated input current based on output current and duty cycle
	double prev_calc_input_current; //Previous calculated input current, used in IncCond MPPT algorithm
	double input_voltage; //Current measured input voltage of substring
	double prev_input_voltage; //Previous input voltage measurement, used in IncCond MPPT algorithm
	uint8_t MPPT_in_progress; //Flag to indicate that the MPPT loop is in progress
	float MPP_voltage; //Voltage of the MPP that was determined in previous MPPT loop
	float MPP_current; //Current of the MPP that was detmerined in previous MPPT loop
};

struct board_param { //Struct to hold values from the board level
	float I_Out;
	float V_Out;
	float V_Out_Voltage_Div_Ratio;

	//MPPT algorithm
	float dV_error; //Tolerance allowed on dV in decision tree of IncCond MPPT algorithm
	float dI_error; //Tolerance allowed on dI in decision tree of IncCond MPPT algorithm

	//Peripherals
	ADC_ChannelConfTypeDef* ADC_config;
	UART_HandleTypeDef* huart_handle;
	ADC_HandleTypeDef* hadc_handle;

	TIM_HandleTypeDef** PWM_TIM;
	uint32_t* PWM_CHANNEL;

	uint32_t* EN_Pins;
	GPIO_TypeDef** EN_Ports;

	TIM_HandleTypeDef* MCU_OK_LED_TIM;
	struct uMPPT** uMPPT_list;
	uint32_t MCU_OK_LED_Period;
	uint32_t MCU_OK_LED_ON_Duration;
	uint32_t* ADC_CH;
};

//------ FUNCTION PROTOTYPES ------//
void config_uMPPT(struct board_param* board);
float uMPPT_read_ADC(uint32_t CH, struct board_param* board);
void update_MPP_IncCond(struct board_param *board, struct uMPPT *target_uMPPT);
void update_MPP_HillClimb(struct board_param *board, struct uMPPT *target_uMPPT);
void updatePWMDutyCycle(struct uMPPT *target_uMPPT, struct board_param* board, float new_duty_cycle);
void updatePWMPhaseOffset(struct uMPPT *target_uMPPT, struct board_param* board, float new_phase_offset);
void updatePWMFreq(struct uMPPT *target_uMPPT, struct board_param* board, float new_freq);
