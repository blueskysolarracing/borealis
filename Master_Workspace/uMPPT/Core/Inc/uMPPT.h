
#ifndef INC_UMPPT_H_
#define INC_UMPPT_H_

#define DEFAULT_PWM_FREQ 			30
#define DEFAULT_PWM_DUTY_CYCLE 		0.8

#define MIN_PWM_DUTY_CYCLE			0.02
#define MAX_PWM_DUTY_CYCLE 			0.99

#define NUM_UMPPT					5

#define ADC_CONV_FACTOR				(float) 8192
#define DEBUG_MODE					1



struct uMPPT {

	uint8_t uMPPT_ID;
	uint8_t mppt_in_progress;
	uint8_t pwm_num;
	double pwm_phase_offset;
	double pwm_duty_cycle;
	double pwm_frequency;
	double calc_input_current;
	double input_voltage;
	double prev_input_voltage;

	float MPP_voltage;
	float MPP_current;

};

struct board_parameters {

	struct uMPPT** uMPPTs;

	float i_out;
	float v_out;
	float v_out_voltage_div_ratio;

	float dV_error;
	float dI_error;

	ADC_ChannelConfTypeDef* ADC_config;
	ADC_HandleTypeDef* hadc_handle;
	uint32_t* ADC_channels;

	UART_HandleTypeDef* huart_handle;
	SPI_HandleTypeDef* hspi_handle;
	TIM_HandleTypeDef** PWM_TIM;
	uint32_t* PWM_CHANNEL;

	GPIO_TypeDef** CS_Ports;
	uint32_t* CS_Pins;

	TIM_HandleTypeDef* MCU_OK_LED_TIM;
	uint32_t MCU_OK_LED_Period;
	uint32_t MCU_OK_LED_ON_Duration;
};

void config_uMPPT(struct board_parameters* board);
float SPI_readADC(uint8_t uMPPT_ID, struct board_parameters* board);
float STM_readADC(struct board_parameters* board);
void update_MPP_IncCond();
void update_MPP_HillClimb();
void updatePWMDutyCycle();
void updatePWMPhaseOffset();
void updatePWMFreq();



#endif /* INC_UMPPT_H_ */
