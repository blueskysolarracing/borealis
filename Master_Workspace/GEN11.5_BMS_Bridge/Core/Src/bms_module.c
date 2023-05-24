/*
 * bms_module.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Raymond
 */
#include "bms_module.h"
#include <math.h>

#define BMS_DISCONNECTED_VOLTAGE_THRESHOLD (6.0f) // any value over 6V indicates BMS Module is disconnected
#define BMS_DISCONNECTED_TEMPERATURE_THRESHOLD (-100.0f) // any value at or below -100C indicates BMS Module is disconnected

static void get_temperature(BmsModule* this, float* temperature, get_mode_t get_mode);
static void get_voltage(BmsModule* this, float* voltage, get_mode_t get_mode);
static void get_state_of_charge(BmsModule* this, float* soc);
static void set_current(BmsModule* this, float current);
static void measure_temperature(BmsModule* this);
static void measure_voltage(BmsModule* this);
static void compute_soc(BmsModule* this);
static inline bool check_voltage_is_valid(float voltage);
static inline bool check_temperature_is_valid(float temperature);

static void sfq_init(StaticFloatQueue* this);
static bool sfq_push(StaticFloatQueue* this, float val);
static bool sfq_peek_idx(StaticFloatQueue* this, float* ret_val, int idx);
static bool sfq_is_empty(StaticFloatQueue* this);
static float sfq_get_avg(StaticFloatQueue* this);

static void LTC6810GeneratePECbits(int CMD0[],int CMD1[], int PCE0[], int PCE1[]);
static void LTC6810GeneratePECbits6Byte(int data6Byte[], int PCE0[], int PCE1[]);
static uint8_t LTC6810ArrayToByte(int arrayIn[]); //8 bit array to a byte
static void LTC6810CommandToArray(int command, int CMD0ref[], int CMD1ref[]);
static void LTC6810CommandGenerate(int command, uint8_t dataToSend[]); //dataToSend is array pass by reference
static void LTC6810CommandGenerateAddressMode(int command, uint8_t dataToSend[], uint8_t address); //dataToSend is array pass by reference
static void LTC6810IsospiWakeup(BmsModule* this);
static void LTC6810TransmitIsospiMode(BmsModule* this, uint8_t dataToSend[], uint8_t dataToSendLen);
static void LTC6810TransmitReceiveIsospiMode(BmsModule* this, uint8_t dataToSend[], uint8_t dataToSendLen, uint8_t dataToReceive[], uint8_t dataToReceiveLen);
static int LTC6810InitializeAndCheck();
static int LTC6810VoltageDataConversion(uint8_t lowByte, uint8_t highByte);
static void LTC6810Init(BmsModule* this, int GPIO4, int GPIO3, int GPIO2 ,int DCC5, int DCC4, int DCC3, int DCC2, int DCC1);
static void LTC6810Convert_to_temp(float input_voltage[3], float output_temperature[3]);
static void LTC6810ReadTemp(BmsModule* this, float* tempArray, int DCC5, int DCC4, int DCC3, int DCC2, int DCC1);
static void LTC6810ReadVolt(BmsModule* this, float* voltArray);

//==== extern the following stuff cuz the test code runs in this .c file ====

extern CRC_HandleTypeDef hcrc;

extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
// ==== end of extern ====


void bms_module_while_loop_test(void* parameters) {
	//BMS test code, when run need to disabling the actual code in main

	//to test:
	//config,including setting gpio & set vref onoff
	//along w/ gpio, test thermal

	//read voltages all 6 channels

	//try balancing? if have time.

	//rest for 1 sec

	float voltarrayTest[5];
	char MSG[50];


	BmsModule* this = (BmsModule*)parameters;
	while(this->init_flag == 0){
	}
	LTC6810IsospiWakeup(this);
	LTC6810Init(this,0,0,0,0,0,0,0,0);

	while (1) {

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, RESET);

		uint8_t dataToSend[16] = {0};//normal transmit use 2byte data 2byte PEC. Config use 6 byte data and 2byte PEC
		int commandInBinary = 0b01001100000;
		uint8_t address = 0;

		//LTC6810CommandGenerateAddressMode(commandInBinary, dataToSend, address);
		//LTC6810TransmitIsospiMode(this, dataToSend, 4);
		LTC6810ReadVolt(this, voltarrayTest);

		 sprintf(MSG, "return0 = %.2f ,%.2f, %.2f,%.2f,%.2f  .\r\n ", voltarrayTest[0],voltarrayTest[1], voltarrayTest[2], voltarrayTest[3],voltarrayTest[4],voltarrayTest[5]);
		  HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		vTaskDelay(1000);


	}
}

void bms_module_init(
		BmsModule* this,
		int bms_module_id,
		SPI_HandleTypeDef* spi_handle,
		GPIO_TypeDef* spi_cs_port,
		uint16_t spi_cs_pin
) {
	this->init_flag = 0;
	this->get_temperature = get_temperature;
	this->get_voltage = get_voltage;
	this->get_state_of_charge = get_state_of_charge;
	this->set_current = set_current;
	this->measure_temperature = measure_temperature;
	this->measure_voltage = measure_voltage;
	this->compute_soc = compute_soc;

	this->_bms_module_id = bms_module_id;

	this->_spi_handle = spi_handle;
	this->_spi_cs_port = spi_cs_port;
	this->_spi_cs_pin = spi_cs_pin;

	this->_current = 0;

	HAL_GPIO_WritePin(this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_SET);

	for (int i = 0; i < BMS_MODULE_NUM_VOLTAGES; i++) {
		sfq_init(&this->past_voltages[i]);
		this->_state_of_charges[i] = BATTERY_SOC_INITIAL_VALUE;
		this->_voltages[i] = BATTERY_CELL_VOLTAGES_INITIAL_VALUE;
	}
	for (int i = 0; i < BMS_MODULE_NUM_TEMPERATURES; i++) {
		sfq_init(&this->past_temperatures[i]);
		this->_temperatures[i] = BATTERY_TEMPERATURES_INITIAL_VALUE;
	}

	measure_voltage(this);
	measure_temperature(this);

	//--- SOC algorithm ---//
	for (int i = 0; i < BMS_MODULE_NUM_STATE_OF_CHARGES; i++){
		this->_tick_last_soc_compute[i] = xTaskGetTickCount();
		initBatteryAlgo(&this->_EKF_models[i], this->_voltages[i], this->_tick_last_soc_compute[i]);
	}

	compute_soc(this);

	this->init_flag = 1;
}

static void get_temperature(BmsModule* this, float* temperatures, get_mode_t get_mode)
{
	vTaskSuspendAll();
	for (int i = 0; i < BMS_MODULE_NUM_TEMPERATURES; i++){
		if (get_mode == GET_MOST_RECENT) {
			temperatures[i] = this->_temperatures[i];
		} else if (get_mode == GET_PAST_AVERAGE) {
			if (check_temperature_is_valid(this->_temperatures[i])) {
				temperatures[i] = sfq_get_avg(&this->past_temperatures[i]);
			} else {
				temperatures[i] = BATTERY_TEMPERATURES_INITIAL_VALUE;
			}
		} else if (get_mode == GET_FILTERED_RESULT) {
			// Can consider running past temperatures through Moving Average Filter, not necessary for now
		}
	}
	xTaskResumeAll();
}

static void get_voltage(BmsModule* this, float* voltages, get_mode_t get_mode)
{
	vTaskSuspendAll();
	for (int i = 0; i < BMS_MODULE_NUM_VOLTAGES; i++){
		if (get_mode == GET_MOST_RECENT) {
			voltages[i] = this->_voltages[i];
		} else if (get_mode == GET_PAST_AVERAGE) {
			if (check_voltage_is_valid(this->_voltages[i])) {
				voltages[i] = sfq_get_avg(&this->past_voltages[i]);
			} else {
				voltages[i] = BATTERY_CELL_VOLTAGES_INITIAL_VALUE;
			}
		} else if (get_mode == GET_FILTERED_RESULT) {
			// Can consider running past voltages through Moving Average Filter, not necessary for now
		}
	}
	xTaskResumeAll();
}

static void get_state_of_charge(BmsModule* this, float* state_of_charges)
{
	vTaskSuspendAll();
	for (int i = 0; i < BMS_MODULE_NUM_STATE_OF_CHARGES; i++){
		state_of_charges[i] = this->_state_of_charges[i];
	}
	xTaskResumeAll();
}

static void set_current(BmsModule* this, float current)
{
	this->_current = current;
}

static void measure_temperature(BmsModule* this)
{
	float local_temp_array[BMS_MODULE_NUM_TEMPERATURES]; //Temperature readings

	// Note the following function actually reads voltage
	LTC6810ReadTemp(this, local_temp_array, 0, 0, 0, 0, 0); //Places temperature in temp 1, temp

	// The voltage needs to be converted to temperature
	LTC6810Convert_to_temp(local_temp_array, local_temp_array);

	vTaskSuspendAll();
	for (int i = 0; i < BMS_MODULE_NUM_TEMPERATURES; i++) {
		if (check_temperature_is_valid(local_temp_array[i])) {
			this->_temperatures[i] = local_temp_array[i];
			if (sfq_is_empty(&this->past_temperatures[i])) {
				for (int j = 0; j < STATIC_FLOAT_QUEUE_NUM_VALUES; j++) {
					sfq_push(&this->past_temperatures[i], this->_temperatures[i]);
				}
			} else {
				sfq_push(&this->past_temperatures[i], this->_temperatures[i]);
			}
		} else {
			// If temperature is invalid, assume bms module is not connected, and don't push to queue
			this->_temperatures[i] = BATTERY_TEMPERATURES_INITIAL_VALUE;
		}
	}
	xTaskResumeAll();

}

static void measure_voltage(BmsModule* this)
{
	float local_voltage_array[BMS_MODULE_NUM_VOLTAGES]; //Voltage of each cell
	LTC6810ReadVolt(this, local_voltage_array); //Places voltage in temp 1, temp

	vTaskSuspendAll();
	for (int i = 0; i < BMS_MODULE_NUM_VOLTAGES; i++) {
		if (check_voltage_is_valid(local_voltage_array[i])) {
			this->_voltages[i] = local_voltage_array[i];
			if (sfq_is_empty(&this->past_voltages[i])) {
				for (int j = 0; j < STATIC_FLOAT_QUEUE_NUM_VALUES; j++) {
					sfq_push(&this->past_voltages[i], this->_voltages[i]);
				}
			} else {
				sfq_push(&this->past_voltages[i], this->_voltages[i]);
			}
		} else {
			// If voltage is invalid, assume bms module is not connected, and don't push to queue
			this->_voltages[i] = BATTERY_CELL_VOLTAGES_INITIAL_VALUE;
		}
	}
	xTaskResumeAll();
}

static void compute_soc(BmsModule* this)
{
	float local_soc_array[BMS_MODULE_NUM_STATE_OF_CHARGES];

	for (int i = 0; i < BMS_MODULE_NUM_STATE_OF_CHARGES; i++) {
		if (check_voltage_is_valid(this->_voltages[i])) {
			uint32_t tick_now = xTaskGetTickCount();
//			this->_EKF_models[i].run_EKF(
//					&this->_EKF_models[i],
//					tick_now - this->_tick_last_soc_compute[i],
//					this->_current,
//					sfq_get_avg(&this->past_voltages[i])
//			);
//			local_soc_array[i] = this->_EKF_models[i].stateX[0];
			this->_tick_last_soc_compute[i] = tick_now;
			local_soc_array[i] = SOC(sfq_get_avg(&this->past_voltages[i]));
		} else {
			// If voltage is invalid, assume bms module is not connected
			local_soc_array[i] = BATTERY_SOC_INITIAL_VALUE;
		}
	}

	vTaskSuspendAll();
	for (int i = 0; i < BMS_MODULE_NUM_STATE_OF_CHARGES; i++){
		this->_state_of_charges[i] = local_soc_array[i];
	}
	xTaskResumeAll();
}

static inline bool check_voltage_is_valid(float voltage)
{
	return voltage < BMS_DISCONNECTED_VOLTAGE_THRESHOLD
			&& voltage != BATTERY_CELL_VOLTAGES_INITIAL_VALUE;
}

static inline bool check_temperature_is_valid(float temperature)
{
	return temperature > BMS_DISCONNECTED_TEMPERATURE_THRESHOLD
			&& temperature != BATTERY_TEMPERATURES_INITIAL_VALUE;
}


static void sfq_init(StaticFloatQueue* this)
{
	q_init_static(
		&this->q,
		sizeof(float),
		STATIC_FLOAT_QUEUE_NUM_VALUES,
		FIFO,
		true, // Overwrite old values (so we only need to call push() and no need to pop())
		this->vals,
		sizeof(this->vals)
	);
}

static bool sfq_push(StaticFloatQueue* this, float val)
{
	return q_push(&this->q, &val);
}

static bool sfq_peek_idx(StaticFloatQueue* this, float* ret_val, int idx)
{
	return q_peekIdx(&this->q, ret_val, idx);
}

static bool sfq_is_empty(StaticFloatQueue* this)
{
	return q_isEmpty(&this->q);
}

static float sfq_get_avg(StaticFloatQueue* this)
{
	float total_past_vals = 0;
	int num_past_vals = 0;
	for (int idx = 0; idx < STATIC_FLOAT_QUEUE_NUM_VALUES; idx++){
		float past_val;
		if (sfq_peek_idx(this, &past_val, idx)) { // evaluates to true if val is available
			num_past_vals++;
			total_past_vals += past_val;
		}
	}
	return total_past_vals / (float)num_past_vals;
}

static void LTC6810Init(BmsModule* this, int GPIO4, int GPIO3, int GPIO2 ,int DCC5, int DCC4, int DCC3, int DCC2, int DCC1) {
	//This function initialize the LTC6810 ADC chip, shall be called at the start of the program
	//To modify the initialization setting, please refer to the Datasheet and the chart below
	/*
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | reg   | Bit7    | Bit6    | Bit5    | Bit4    | Bit3    | Bit2    | Bit1    | Bit0   |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR0 | RSVD    | GPIO4   | GPIO3   | GPIO2   | GPIO1   | REFON   | DTEN    | ADCOPT |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR1 | VUV[7]  | VUV[6]  | VUV[5]  | VUV[4]  | VUV[3]  | VUV[2]  | VUV[1]  | VUV[0] |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR2 | VOV[3]  | VOV[2]  | VOV[1]  | VOV[0]  | VUV[11] | VUV[10] | VUV[9]  | VUV[8] |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR3 | VOV[11] | VOV[10] | VOV[9]  | VOV[8]  | VOV[7]  | VOV[6]  | VOV[5]  | VOV[4] |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR4 | DCC0    | MCAL    | DCC6    | DCC5    | DCC4    | DCC3    | DCC2    | DCC1   |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+
	 | CFGR5 | DCTO[3] | DCTO[2] | DCTO[1] | DCTO[0] | SCONV   | FDRF    | DIS_RED | DTMEN  |
	 +-------+---------+---------+---------+---------+---------+---------+---------+--------+*/

	//connection to MUX for thermal measurements:
	//GPIO2 -> A0
	//GPIO3 -> A1
	//GPIO4 -> A2
	//GPIO1 shall always be set to 1 to avoid internal pull-down, as its the voltage reading pin.
	//A2 is MSB and A0 is MSB, if want channel 2 -> do A2=0, A1=1, A0=0
	//above is the table of configuration register bits. How
	int messageInBinary;
	int i;
	int data6Byte[48]; //[47] is MSB, CFGR0 Bit7
	uint8_t dataToSend[12]; //stores the command and PEC bit of WRCFG
	//2byte command, 2byte PEC, 6byte data, 2byte PEC

	//write configure command
	messageInBinary = 0b1;		//write config
	//LTC6810CommandGenerate(messageInBinary, dataToSend);
	LTC6810CommandGenerateAddressMode(messageInBinary, dataToSend, this->_bms_module_id);
	//set GPIO bits to 1 so they aren`t being pulled down internally by the chip.
	//set REFON to enable the 3V that goes to the chip
	//DTEN to 1 to enable discharge timer
	//ADCOPT bit to 0, use 422Hz as its stable
	//above are byte0, byte 1 full of 0s as VUV currently not used.
	messageInBinary = 0b0000110000000000; //CFGR0&1
	//now add the GPIO config
	messageInBinary = messageInBinary + 4096 * GPIO2 + 8192 * GPIO3
			+ 16384 * GPIO4;

	int MSG0[8];
	int MSG1[8];
	LTC6810CommandToArray(messageInBinary, MSG0, MSG1);
	for (i = 7; i >= 0; i--) {
		data6Byte[40 + i] = MSG0[i];
		data6Byte[32 + i] = MSG1[i];
	}

	//set the VOV and VUV as 0;
	messageInBinary = 0b0000000000000000; //CFGR2&3
	int MSG2[8];
	int MSG3[8];
	LTC6810CommandToArray(messageInBinary, MSG2, MSG3);
	for (i = 7; i >= 0; i--) {
		data6Byte[24 + i] = MSG2[i];
		data6Byte[16 + i] = MSG3[i];
	}

	//DCC = 0: discharge off, 1: discharge = on
	//MCAL = 1: enable multi-calibration. for this don`t use, turn to 0
	//DCTO: discharge timer. for now 0
	//SCONV: redundant measurement using S pin, disable for now (0)
	//FDRF: not using it anyway, to 0
	//DIS_RED: redundancy disable: set to 1 to disable
	//DTMEN: Discharge timer monitor, 0 to disable
	messageInBinary = 0b0000000000000010;
	messageInBinary += 256*DCC1 + 512*DCC2 + 1024*DCC3 + 2048*DCC4 +4096*DCC5;

	int MSG4[8];
	int MSG5[8];
	LTC6810CommandToArray(messageInBinary, MSG4, MSG5);
	for (i = 7; i >= 0; i--) {
		data6Byte[8 + i] = MSG4[i];
		data6Byte[0 + i] = MSG5[i];
	}

	// //now create PEC bits based on above data
	int PEC0_6[8];
	int PEC1_6[8];
	LTC6810GeneratePECbits6Byte(data6Byte, PEC0_6, PEC1_6);

	// //change array back to bytes
	dataToSend[4] = LTC6810ArrayToByte(MSG0);
	dataToSend[5] = LTC6810ArrayToByte(MSG1);
	dataToSend[6] = LTC6810ArrayToByte(MSG2);
	dataToSend[7] = LTC6810ArrayToByte(MSG3);
	dataToSend[8] = LTC6810ArrayToByte(MSG4);
	dataToSend[9] = LTC6810ArrayToByte(MSG5);
	dataToSend[10] = LTC6810ArrayToByte(PEC0_6);
	dataToSend[11] = LTC6810ArrayToByte(PEC1_6);

	//now send the data
	LTC6810TransmitIsospiMode(this, dataToSend, 12);

}


/*======plz read the stuff below and it will save time
 * there are 3 low level functions:
 * 	1.LTC6810CommandToArray 2.LTC6810GeneratePECbits
 * the function  LTC6810CommandGenerate(int command, uint8_t dataToSend[]) is the
 * function that u want to use in main.c, other functions are all supporting functions.
 *
 * the LTC6810checkAllVoltage(int voltageArray[]) is a function
 *
 */

static void LTC6810GeneratePECbits(int CMD0[],int CMD1[], int PCE0[], int PCE1[]){

	//refer to P55 of LTC
	int IN0,IN3,IN4,IN7,IN8,IN10,IN14;
	IN0=IN3=IN4=IN7=IN8=IN10=IN14=0; //initialize all polynomial stuff to 0

	int PEC[15];
	int i=0;
	while(i<15){
		PEC[i] = 0;
		i++;
	}
	PEC[4] = 1;
	//above initialize the 15 bit PEC. in last cycle shift up add 0 at LSB

	//Count Din from most significant bit
	int DIN;//
	for(i=15;i >= 0; i--){//do the shifting and stuff

		if(i>=8){//grab DIN from CMD0
			DIN = CMD0[i-8];
		}
		else{
			DIN = CMD1[i];
		}
		//now DIN is assigned, do step 2 which is shifting
		IN0 = DIN ^ PEC[14];
		IN3 = IN0 ^ PEC [2];
		IN4 = IN0 ^ PEC [3];
		IN7 = IN0 ^ PEC [6];
		IN8 = IN0 ^ PEC [7];
		IN10 = IN0 ^ PEC [9];
		IN14 = IN0 ^ PEC [13];

		//step 3, shift
		PEC [14] = IN14;
		PEC [13] = PEC [12];
		PEC [12] = PEC [11];
		PEC [11] = PEC [10];
		PEC [10] = IN10;
		PEC [9] = PEC [8];
		PEC [8] = IN8;
		PEC [7] = IN7;
		PEC [6] = PEC [5];
		PEC [5] = PEC [4];
		PEC [4] = IN4;
		PEC [3] = IN3;
		PEC [2] = PEC [1];
		PEC [1] = PEC [0];
		PEC [0] = IN0;

		//printf(" %d \n",i);
		//printf("Din: %d 0:%d 3:%d 4:%d 7:%d 8:%d 10:%d 14:%d \n",DIN,IN0,IN3,IN4,IN7,IN8,IN10,IN14 );

	}//end of main shifting for loop
	int j;
	for(j=14;j>=0;j--){
	    //printf("%d",PEC[j]);   ///test
	    if(j>=7){
	       PCE0[j-7] = PEC[j];
	    }
	    else{
	        PCE1[j+1] = PEC[j];
	    }
	}
	//now add a 0 at the end and stuff it into 2 bytes
	PCE1[0]=0;

}//====== end of generatePCEbits function =========

static void LTC6810GeneratePECbits6Byte(int data6Byte[], int PCE0[], int PCE1[]){
    //Note: data6Byte[74] is 1st bit

	//refer to P55 of LTC
	int IN0,IN3,IN4,IN7,IN8,IN10,IN14;
	IN0=IN3=IN4=IN7=IN8=IN10=IN14=0; //initialize all polynomial stuff to 0

	int PEC[15];
	int i=0;
	while(i<15){
		PEC[i] = 0;
		i++;
	}
	PEC[4] = 1;
	//above initialize the 15 bit PEC. in last cycle shift up add 0 at LSB

	//Count Din from most significant bit
	int DIN;//
	for(i=47;i >= 0; i--){//do the shifting and stuff

		DIN = data6Byte[i];
		//now DIN is assigned, do step 2 which is shifting
		IN0 = DIN ^ PEC[14];
		IN3 = IN0 ^ PEC [2];
		IN4 = IN0 ^ PEC [3];
		IN7 = IN0 ^ PEC [6];
		IN8 = IN0 ^ PEC [7];
		IN10 = IN0 ^ PEC [9];
		IN14 = IN0 ^ PEC [13];

		//step 3, shift
		PEC [14] = IN14;
		PEC [13] = PEC [12];
		PEC [12] = PEC [11];
		PEC [11] = PEC [10];
		PEC [10] = IN10;
		PEC [9] = PEC [8];
		PEC [8] = IN8;
		PEC [7] = IN7;
		PEC [6] = PEC [5];
		PEC [5] = PEC [4];
		PEC [4] = IN4;
		PEC [3] = IN3;
		PEC [2] = PEC [1];
		PEC [1] = PEC [0];
		PEC [0] = IN0;

		//printf(" %d \n",i);
		//printf("Din: %d 0:%d 3:%d 4:%d 7:%d 8:%d 10:%d 14:%d \n",DIN,IN0,IN3,IN4,IN7,IN8,IN10,IN14 );

	}//end of main shifting for loop
	int j;
	for(j=14;j>=0;j--){
	    //printf("%d",PEC[j]);   ///test
	    if(j>=7){
	       PCE0[j-7] = PEC[j];
	    }
	    else{
	        PCE1[j+1] = PEC[j];
	    }
	}
	//now add a 0 at the end and stuff it into 2 bytes
	PCE1[0] = 0;

}//====== end of LTC6810GeneratePECbits6Byte function=====


uint8_t LTC6810ArrayToByte(int arrayIn[]){ //8 bit array to a byte
	uint8_t outputByte = 0;
	int i=7;
	//note: [7] is MSB, [0] is LSB.
	while(i>=1){
		outputByte = outputByte + arrayIn[i];
		outputByte = outputByte * 2; //left shift one bit
		i--;
		//note: arrayIn[i] shall be either 1 or 0;
	}
	outputByte = outputByte + arrayIn[i];//put last bit in
	return outputByte;
}//======== end of LTC6810ArrayToByte function ========



static void LTC6810CommandToArray(int command, int CMD0ref[], int CMD1ref[]){
	int tempBit;//stores the temporary bit value from command that goes into CMD0 or 1

	int i;
	for(i = 0; i<=15;i++){ //from LSB to MSB, fill CMD1 first
		tempBit = (command % 2 );
		if(i<=7){
			CMD1ref[i] = tempBit;
		}
		else{
			CMD0ref[i-8] = tempBit;
		}
		command = command >> 1; //right shift to remove the LSB
		// the ">>" is a binary rightshift command
	}

}//======== end of LTC6810CommandToArray function ========


static void LTC6810CommandGenerate(int command, uint8_t dataToSend[]){ //dataToSend is array pass by reference
	int CMD0Array[8] = {0,0,0,0,0,0,0,0};
	int CMD1Array[8] = {0,0,0,0,0,0,0,0};//[7] is MSB, [0] is LSB
	//convert command and fill the two above array
	LTC6810CommandToArray(command, CMD0Array, CMD1Array);

	//now generate Error checking bits
	int PECbits0[8] = {0,0,0,0,0,0,0,0};
	int PECbits1[8] = {0,0,0,0,0,0,0,0};

	LTC6810GeneratePECbits(CMD0Array, CMD1Array, PECbits0, PECbits1);

	//put them back into the dataToSend array, with size of 4

	dataToSend[0] = LTC6810ArrayToByte(CMD0Array);//1st byte message
	dataToSend[1] = LTC6810ArrayToByte(CMD1Array);//2nd byte message

	dataToSend[2] = LTC6810ArrayToByte(PECbits0);//1st byte error check
	dataToSend[3] = LTC6810ArrayToByte(PECbits1);//2nd byte error check

} // ======== end of LTC6810CommandGenerate function ==========

static void LTC6810CommandGenerateAddressMode(int command, uint8_t dataToSend[], uint8_t address){ //dataToSend is array pass by reference
	int addressCommand = (0b10000 | (address & 0b1111)) << 11;
	command |= addressCommand;
	LTC6810CommandGenerate(command, dataToSend);
}

static void LTC6810IsospiWakeup(BmsModule* this){
	//No delay more than 3ms allowed (between this function call and the actual SPI comm)! else isoSPI might go back to sleep
	// Need to transmit dummy byte
	uint8_t dummyByte[1] = {'0'};
	HAL_GPIO_WritePin(this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_RESET); // Falling edge to trigger wake up
	HAL_SPI_Transmit(this->_spi_handle, dummyByte, 1, 100);
	HAL_GPIO_WritePin(this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_SET);
}

static void LTC6810TransmitIsospiMode(BmsModule* this, uint8_t dataToSend[], uint8_t dataToSendLen){
	//directly trans
	LTC6810IsospiWakeup(this);
	vTaskDelay(pdMS_TO_TICKS(1));//this 1ms delay is crucial
	HAL_GPIO_WritePin(this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_RESET); // Create falling edge
	vTaskDelay(pdMS_TO_TICKS(1)); //require a few us. We delay by 1 ms, since this is the smallest interval we can set
	HAL_SPI_Transmit(this->_spi_handle, dataToSend, dataToSendLen, 100);
	HAL_GPIO_WritePin(this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_SET);
}

static void LTC6810TransmitReceiveIsospiMode(BmsModule* this, uint8_t dataToSend[], uint8_t dataToSendLen, uint8_t dataToReceive[], uint8_t dataToReceiveLen){
	LTC6810IsospiWakeup(this);
	vTaskDelay(pdMS_TO_TICKS(1));//this 1ms delay is crucial
	HAL_GPIO_WritePin(this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_RESET); // Create falling edge
	vTaskDelay(pdMS_TO_TICKS(1)); //require a few us. We delay by 1 ms, since this is the smallest interval we can set
	HAL_SPI_Transmit(this->_spi_handle, dataToSend, dataToSendLen, 100);
	vTaskDelay(pdMS_TO_TICKS(1)); //add delay between Transmit & Receive
	HAL_SPI_Receive(this->_spi_handle, dataToReceive, dataToReceiveLen, 100);
	vTaskDelay(pdMS_TO_TICKS(1)); // Might not be necessary
	HAL_GPIO_WritePin(this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_SET);
}

/*
static void LTC6810checkAllVoltage(int voltageArray[]){//Array of 6 voltages
	//voltafeArray[1] is voltage of cell 0; [5] is voltage of cell 6
	//also this function will take a bit time cuz the ADC thing takes time

	static int LTC6810Command;  //this variable carries the commands, see on page 60 of datasheet
	uint8_t dataToSend[4]; //converted from the binary command to data bytes
	uint8_t dataToReceive[8];

	LTC6810Command = 0b01011000000; //ADCV of cell 1;
	//MD = 01, 27kHz;  DCP=0, no discharge permit; Channel = 000: All channel;
	LTC6810CommandGenerate(LTC6810Command, dataToSend );//array passed by reference

	HAL_GPIO_WritePin (this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_RESET); //slave select low, transmit begin
	HAL_SPI_Transmit(this->_spi_handle, dataToSend,4,100);
	vTaskDelay(10);//ADD 10ms DELAY between Transmit & Receive
	HAL_SPI_Receive(this->_spi_handle, dataToReceive,4,100);         //discard this Receive as it is problematic
	HAL_GPIO_WritePin (this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_SET);

	vTaskDelay(5);

	//after letting it read ADC, now collect those data

	LTC6810Command = 0b00000000100;
		//RDCVA read cell voltage register group A (cell 1-3)
	LTC6810CommandGenerate(LTC6810Command, dataToSend );
	HAL_GPIO_WritePin (this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_RESET); //slave select low, transmit begin
	HAL_SPI_Transmit(this->_spi_handle, dataToSend,4,100);
	vTaskDelay(2);
	HAL_SPI_Receive(this->_spi_handle, dataToReceive,6,100);         //read 6 bytes, cell0,1,2
	HAL_GPIO_WritePin (this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_SET); //ss pin back high

	voltageArray[0] = (dataToReceive[1]*256) + dataToReceive[0];
	//lsb comes first, refer to page 64 Memory map
	voltageArray[1] = (dataToReceive[3]*256) + dataToReceive[2];
	voltageArray[2] = (dataToReceive[5]*256) + dataToReceive[4];

	LTC6810Command = 0b00000000110;
		//RDCVA read cell voltage register group B (cell 4-6)
	LTC6810CommandGenerate(LTC6810Command, dataToSend );
	HAL_GPIO_WritePin (this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_RESET); //slave select low, transmit begin
	HAL_SPI_Transmit(this->_spi_handle, dataToSend,4,100);
	vTaskDelay(2);
	HAL_SPI_Receive(this->_spi_handle, dataToReceive,6,100);         //read 6 bytes, cell0,1,2
	HAL_GPIO_WritePin (this->_spi_cs_port, this->_spi_cs_pin, GPIO_PIN_SET); //ss pin back high

	voltageArray[3] = (dataToReceive[1]*256) + dataToReceive[0];
	voltageArray[4] = (dataToReceive[3]*256) + dataToReceive[2];
	voltageArray[5] = (dataToReceive[5]*256) + dataToReceive[4];


}*/

static int LTC6810InitializeAndCheck(){
	//Call this function when starting up.
/*
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| reg   | Bit7    | Bit6    | Bit5    | Bit4    | Bit3    | Bit2    | Bit1    | Bit0   |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR0 | RSVD    | GPIO4   | GPIO3   | GPIO2   | GPIO1   | REFON   | DTEN    | ADCOPT |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR1 | VUV[7]  | VUV[6]  | VUV[5]  | VUV[4]  | VUV[3]  | VUV[2]  | VUV[1]  | VUV[0] |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR2 | VOV[3]  | VOV[2]  | VOV[1]  | VOV[0]  | VUV[11] | VUV[10] | VUV[9]  | VUV[8] |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR3 | VOV[11] | VOV[10] | VOV[9]  | VOV[8]  | VOV[7]  | VOV[6]  | VOV[5]  | VOV[4] |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR4 | DCC0    | MCAL    | DCC6    | DCC5    | DCC4    | DCC3    | DCC2    | DCC1   |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+
| CFGR5 | DCTO[3] | DCTO[2] | DCTO[1] | DCTO[0] | SCONV   | FDRF    | DIS_RED | DTMEN  |
+-------+---------+---------+---------+---------+---------+---------+---------+--------+*/

//above is the table of configuration register bits. How
//	static int LTC6810Command;
//	uint8_t dataToSend0[4]; //stores the command and PEC bit of WRCFG
//	uint8_t dataToSend1[4]; //stores CFGR0 & CFGR1 stuff
//	uint8_t dataToSend2[4]; //stores byte 2 & 3
//	uint8_t dataToSend3[4]; //stores byte 4&5
//	uint8_t dataToReceive[8];
//set GPIO bits to 1 so they aren`t being pulled down internally by the chip.
//set REFON to enable the 3V that goes to the chip
//DTEN to 0 to disable discharge timer
//ADCOPT bit to 0, use 422Hz as its stable
//above are byte0, byte 1 full of 0s as VUV currently not used.


return 0;//temp

}//====== End of LTC6810initializeAndCheck ======

static int LTC6810VoltageDataConversion(uint8_t lowByte, uint8_t highByte){
	//based on LTC6810 datasheet, every bit is 100uV
	//full range of 16 bytes: -0.8192 to + 5.7344
	//Final voltage: (total - 8192) * 100uV
	int Voltage;
	Voltage = lowByte + highByte * 256;
	//Voltage = Voltage + 8192;
	return Voltage;
}

static void LTC6810Convert_to_temp(float input_voltage[3], float output_temperature[3]){
	/* Used to convert raw ADC code to temperature */
	float temp_correction_multiplier = 1.0;
	float temp_correction_offset = 0.0;
	for (int i = 0; i < BMS_MODULE_NUM_TEMPERATURES; i++){
		float corrected_voltage = temp_correction_multiplier * (input_voltage[i] / 10000.0  - temp_correction_offset);
		float thermistor_resistance = 10.0 / ((2.8 / (float) corrected_voltage) - 1.0);
		output_temperature[i] = 1.0 / (0.003356 + 0.0002532 * log(thermistor_resistance / 10.0));
		output_temperature[i] = output_temperature[i] - 273.15;

		if (thermistor_resistance < 0) {  //Preventing NaN (treated as 0xFFFFFFFF) from tripping the battery safety but lower-bounding them to -100C
			output_temperature[i] = -100;
		}
		//This occurs when the measurement is closes to 3V, suggesting that the thermistor isn't connected.
	}
}

static void LTC6810ReadTemp(BmsModule* this, float* tempArray, int DCC5, int DCC4, int DCC3, int DCC2, int DCC1){
	//read Temp 0,1,2. Pass by reference to the input array.
	//if discharging, make DCC global variables so this don't disturb Discharge

	int cycle = 0;
	int messageInBinary;
	uint8_t dataToSend[16];
	uint8_t dataToReceive[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };//voltage data from LTC6810 via SPI
	while(cycle<3){

		if(cycle ==0){ //channel 3
			LTC6810Init(this, 0,1,1,DCC5, DCC4, DCC3, DCC2, DCC1);
		}
		else if(cycle == 1){ //channel 4
			LTC6810Init(this, 1,0,0,DCC5, DCC4, DCC3, DCC2, DCC1);
		}
		else if(cycle == 2){ //channel 5
			LTC6810Init(this, 1,0,1,DCC5, DCC4, DCC3, DCC2, DCC1);
		}

		messageInBinary = 0b10100010010;  //conversion GPIO1, command AXOW
		LTC6810CommandGenerateAddressMode(messageInBinary, dataToSend, this->_bms_module_id);
		LTC6810TransmitIsospiMode(this, dataToSend, 4);

		//now read from it
		messageInBinary = 0b1100; //read auxiliary group 1, command RDAUXA
		LTC6810CommandGenerateAddressMode(messageInBinary, dataToSend, this->_bms_module_id);
		LTC6810TransmitReceiveIsospiMode(this, dataToSend, 4, dataToReceive, 8);

		//write value into array
		int tempSum = 256*dataToReceive[3] + dataToReceive[2];
		//float TempInC = 1/(1/298.15+(1/3950) * log((-10)/(tempSum/)))
		tempArray[cycle] = (float)tempSum;//msb of data
		cycle++;
	}
}

static void LTC6810ReadVolt(BmsModule* this, float* voltArray){
	int VmessageInBinary;//for internal command
	uint8_t dataToSend[4]; //data organized by function [LTC6810CommandGenerate]
	uint8_t dataToReceive[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };//voltage data from LTC6810 via SPI

	//first read first half of data
	VmessageInBinary = 0b01101110000; //adcv discharge enable,7Hz
	LTC6810CommandGenerateAddressMode(VmessageInBinary, dataToSend, this->_bms_module_id);//generate the "check voltage command"
	LTC6810TransmitReceiveIsospiMode(this, dataToSend, 4, dataToReceive, 8);

	vTaskDelay(10);
	VmessageInBinary = 0b100;  //read cell voltage reg group 1;
	LTC6810CommandGenerateAddressMode(VmessageInBinary, dataToSend, this->_bms_module_id);
	LTC6810TransmitReceiveIsospiMode(this, dataToSend, 4, dataToReceive, 8);
	voltArray[0] = LTC6810VoltageDataConversion(dataToReceive[0], dataToReceive[1]) /10000.0;
	voltArray[1] = LTC6810VoltageDataConversion(dataToReceive[2], dataToReceive[3]) /10000.0;
	voltArray[2] = LTC6810VoltageDataConversion(dataToReceive[4], dataToReceive[5]) /10000.0;

	VmessageInBinary = 0b110; //read cell voltage reg group 2;
	LTC6810CommandGenerateAddressMode(VmessageInBinary, dataToSend, this->_bms_module_id);
	LTC6810TransmitReceiveIsospiMode(this, dataToSend, 4, dataToReceive, 8);
	voltArray[3] = LTC6810VoltageDataConversion(dataToReceive[0], dataToReceive[1]) /10000.0;
	voltArray[4] = LTC6810VoltageDataConversion(dataToReceive[2], dataToReceive[3]) /10000.0;

	// Not using the following since voltArray has size of 5 instead of 6
	// voltArray[5] = voltageDataConversion(dataToReceive[4], dataToReceive[5]) /10000.0;
}


