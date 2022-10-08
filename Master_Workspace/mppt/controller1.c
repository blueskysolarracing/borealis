/********************************************************************************************************
USER LICENCE AGREEMENT and DISCLAIMER:
This source code is the property of NanoGrid Ltd.
Any modification made to this source code remains the property of NanoGrid Ltd.
The source code may only be used for specific authorized peak powers.
The source code is confidential and must be protected and must not be distributed.
NanoGrid Ltd assumes no liability for damage or harm or losses caused by this source code.
Deviation from this agreement requires permission from NanoGrid Ltd.
This agreement must appear at the top of each source file and applies to the source as well
 as accompanying binary, object, library, or any related files.
Cantact: Tom Rodinger, tom@nanoleaf.me
********************************************************************************************************/

#include <p24hj64gp502.h>
#include "types.h"

_FBS( RBS_NO_RAM & BSS_NO_FLASH & BWRP_WRPROTECT_OFF )
_FSS( RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF )
_FGS( GSS_OFF & GCP_OFF & GWRP_OFF )
_FOSCSEL( FNOSC_FRC & IESO_OFF )
_FOSC( FCKSM_CSECME & IOL1WAY_ON & OSCIOFNC_ON & POSCMD_XT )
_FWDT( FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS32768 )
_FPOR( ALTI2C_OFF & FPWRT_PWR128 )
_FICD( JTAGEN_OFF & ICS_PGD3 )

#define VPANEL_CALIBRATION 32478
#define VBATTERY_CALIBRATION 34862
#define IPANEL_CALIBRATION 60357
#define PPANEL_CALIBRATION ((uint16)((uint32)VBATTERY_CALIBRATION/2*IPANEL_CALIBRATION/128000))

extern void set_up_clock();
extern void ADC_init();
extern void input_capture_init(void);
extern void timing_info_init(void);
extern void input_PWM_init(void);
extern void input_PWM_disable(void);
extern void PPT_init(void);
extern void timekeeper_init(void);
extern int16 wait_for_timekeeper_event();
extern void UART_init(void);
extern void UART_receive(void);
extern void UART_send_number(uint16 num, uint8 dec_loc);
extern void UART_send_divider(void);
extern void UART_send(void);
extern void UART_queue_binary(uint32 value, uint8 length);
extern void UART_start_transmission(void);
extern void UART_process(void);
extern uint16 read_ppt_node_id(void);
extern void set_ppt_node_id(uint16 ppt_node_id);
extern void write_data_to_flash(uint16 ppt_node_id, uint16 battery_charge_voltage);
extern uint16 read_battery_charge_voltage(void);
extern void CAN_init(uint16 ppt_node_id);
extern void CAN_send(uint16 data1, uint16 data2, uint16 data3, uint16 data4);
extern void CAN_send_status(uint16 status1);
extern int16 CAN_receive_command(void);
extern uint16 CAN_get_payload_voltage(void);
extern uint16 CAN_get_payload_voltage_ADC(void);

extern uint16 Vpanel_desired;   // these variable are used for debugging, will remove later
extern uint16 Iavg;
extern uint16 LSadjust;
extern uint16 integral_error;


#define SNAPSHOT_TAKEN 1
uint16 global_flags;

extern uint16 Ipanel_ADC;
extern uint16 Vpanel_ADC;
extern uint16 Vbatt_ADC;
extern uint16 ADC_accumulator_snapshot_flag;
extern uint32 Vpanel_ADC_accumulator_snapshot;
extern uint32 Ipanel_ADC_accumulator_snapshot;
extern uint32 Ppanel_ADC_accumulator_snapshot;
extern uint32 Vbattery_ADC_accumulator_snapshot;

uint16 snapshot_accumulator_counter;
uint32 Vpanel_ADC_accumulator;
uint32 Ipanel_ADC_accumulator;
uint32 Ppanel_ADC_accumulator;
uint32 Vbattery_ADC_accumulator;
uint16 Vpanel;
uint16 Ipanel;
uint16 Ppanel;
uint16 Vbattery;
uint8 send_sequence;

#define INITIAL_TRACKING_MODE          0
#define BUTTON_PRESSED_MODE            1
#define CHECK_CONDITIONS_TRACKING_MODE 2
#define CONSTANT_VOLTAGE_TRACKING_MODE 3
#define FULL_SCAN_TRACKING_MODE        4
#define DITHERING_TRACKING_MODE        5

#define POWER_ON_DELAY ((uint8)200)
#define MAX_VBATT (150 * 20)  // This value must be the same as in input_power_control.s ; set to 155V
#define MIN_VBATT (50 * 20)  // This value must be the same as in input_power_control.s ; set to 90V
#define MIN_VPANEL_TO_VBATT (1 * 20) // This value must be the same as in input_power_control.s ; set to 15V
                                      //  Must: MIN_VPANEL_TO_VBATT <= MIN_VBATT
#define MIN_VPANEL (5 * 20)  // This value must be the same as in input_power_control.s ; set to 10V

// DEBUG START
//#undef MIN_VBATT
//#undef MIN_VPANEL_TO_VBATT
//#undef MIN_VPANEL
//#define MIN_VBATT (1 * 20)
//#define MIN_VPANEL_TO_VBATT (1 * 20)
//#define MIN_VPANEL (1 * 20)
// DEBUG END

#define MAX_VPANEL_DESIRED (MAX_VBATT - MIN_VPANEL_TO_VBATT)
#define MIN_VPANEL_DESIRED (10 * 20)
#define FULL_SCAN_STEP_SIZE 1
#define PPT_STEP_SIZE 10
#define PERTURBATION_STEP_SIZE 2
#define MIN_PERTURBATION_STEPS 10
#define MAX_PERTURBATION_STEPS 50
#define PPT_LOW_CURRENT_THRESHOLD 50331  // setting for 80mA
#define PPT_MIN_CURRENT_THRESHOLD 3775  // setting for 6mA

#define BUTTON_DELAY1 32  // delay to shut off power section, about 2 seconds
#define BUTTON_DELAY2 64  // delay to program the node tag, about 4 seconds

uint8 tracking_mode;
uint8 power_on_delay;
uint16 constant_voltage;
uint16 battery_charge_voltage;
uint32 Imax_found;
uint32 Imax_found_Vdesired;
int16 PPT_direction;
int16 PPT_count;
int16 perturbation_steps;
int16 perturbation_step_size;
int32 PPT_accumulator;
int16 LED_dither_counter;
uint32 PPT_I_max;
uint32 PPT_I_min;
uint32 PPT_V_max1;
uint32 PPT_V_max2;
uint32 PPT_V_max3;
uint32 PPT_V_max4;
uint16 PPT_V_max;

uint16 button_delay;
uint16 new_ppt_node_id;
uint16 ppt_node_id;
uint16 power_on;

#define CAN_COMMAND_POWER_OFF 0
#define CAN_COMMAND_FULL_SCAN 1
#define CAN_COMMAND_CONSTANT_VOLTAGE 2
#define CAN_COMMAND_SET_BATTERY_CHARGE_VOLTAGE 3
int16 CAN_command;

void push_button_init(void)
{
    __asm__("bset CNPU1, #6"); // enable pullup in the pin
	button_delay = 0;
	new_ppt_node_id = 0;
	power_on = 1;
}

void check_push_button(void)
{
    if((PORTB & 0b100) == 0) {
        input_PWM_disable();
		tracking_mode = INITIAL_TRACKING_MODE;
		button_delay++;
		power_on = 1;
		constant_voltage = 0;
		if(button_delay > BUTTON_DELAY2) {
			if((button_delay & 0b111) == 0) {
				new_ppt_node_id++;     // increase the PPT node tag and flash the LED
				power_on = 1;
		        __asm__("bset LATB, #7");
			}
			else {
		        __asm__("bclr LATB, #7");
			}
		}
		else if(button_delay > BUTTON_DELAY1) {
			power_on = 0;
	        __asm__("bclr LATB, #7");
		}
		else {
	        __asm__("bset LATB, #7");
		}
    }
	else {
		if(button_delay > 0) {
			button_delay = 0;
		    __asm__("bclr LATB, #7");
			if(new_ppt_node_id > 0) {
				ppt_node_id = new_ppt_node_id - 1;
				set_ppt_node_id(ppt_node_id);
				write_data_to_flash(ppt_node_id, battery_charge_voltage);
				new_ppt_node_id = 0;
			}
		}
	}
}

void init_check_power_on_conditions(void)
{
    power_on_delay = 0;
}

uint8 check_power_on_conditions(void)
{
//	return 1;  // DEBUG
    if( (Vbattery <= battery_charge_voltage) && (Vbatt_ADC < MAX_VBATT) && (Vbatt_ADC > MIN_VBATT) && (Vpanel_ADC > MIN_VPANEL)) {
        power_on_delay++;
        if(power_on_delay == POWER_ON_DELAY) {
            return 1;
        }
    }
    else {
        power_on_delay = 0;
    }
    return 0;
}

void set_constant_voltage(uint16 V)
{
	if(V < MIN_VPANEL_DESIRED) {
		V = MIN_VPANEL_DESIRED;
	}
    if(V > Vbatt_ADC - MIN_VPANEL_TO_VBATT) {
        V = Vbatt_ADC - MIN_VPANEL_TO_VBATT;
    }
    if(V > MAX_VPANEL_DESIRED) {
        V = MAX_VPANEL_DESIRED;
    }
    Vpanel_desired = V;
}

void init_full_scan_algorithm(void)
{
    uint16 V;

    V = Vpanel_ADC;
    if(V > Vbatt_ADC - MIN_VPANEL_TO_VBATT) {
        V = Vbatt_ADC - MIN_VPANEL_TO_VBATT;
    }
    if(V > MAX_VPANEL_DESIRED) {
        V = MAX_VPANEL_DESIRED;
    }
    Vpanel_desired = V;
    PPT_direction = -1;
    Imax_found = 0;
    Imax_found_Vdesired = 0;
}

uint8 full_scan_algorithm(uint32 I, uint32 V)
{
    switch(PPT_direction) {
    case -1:
        PPT_direction = -2;
        break;
    case -2:
        if(I > Imax_found) {
            Imax_found = I;
            Imax_found_Vdesired = V;
        }
        if(Vpanel_desired - FULL_SCAN_STEP_SIZE < MIN_VPANEL_DESIRED) {
            Vpanel_desired = (Imax_found_Vdesired >> 9) + ((MIN_PERTURBATION_STEPS * PERTURBATION_STEP_SIZE) >> 1);
            return 1;
        }
        Vpanel_desired -= FULL_SCAN_STEP_SIZE;
        PPT_direction = -1;
        break;
    }
    return 0;
}

void experimental_init_dithering_algorithm(void)
{
    PPT_direction = 1;
    perturbation_steps = MIN_PERTURBATION_STEPS;
    perturbation_step_size = PERTURBATION_STEP_SIZE;
    PPT_count = perturbation_steps;
    PPT_accumulator = 0;
    PPT_I_max = 0;
    PPT_V_max1 = 0;
    PPT_V_max2 = 0;
    PPT_V_max3 = 0;
    PPT_V_max4 = 0;
}

void experimental_dithering_algorithm(uint32 I, uint32 V)
{
    uint16 new_Vpanel_desired;

//  UART_queue_binary(0x01234567, 4);
//  UART_queue_binary(0x89ABCDEF, 4);
    UART_queue_binary(I >> 9, 4);
    UART_queue_binary(V >> 9, 4);
    UART_start_transmission();

    switch(PPT_direction)
    {
    case 1:
        PPT_direction = 2;
        break;
    case 2:
        PPT_direction = 1;
        if(I >= PPT_I_max) {
            PPT_I_max = I;
            PPT_V_max1 = V;
        }

        if(I > PPT_MIN_CURRENT_THRESHOLD) {
            new_Vpanel_desired = Vpanel_desired + perturbation_step_size;
            if(new_Vpanel_desired > Vbatt_ADC - MIN_VPANEL_TO_VBATT) {
                new_Vpanel_desired = Vbatt_ADC - MIN_VPANEL_TO_VBATT;
            }
            if(new_Vpanel_desired > MAX_VPANEL_DESIRED) {
                new_Vpanel_desired = MAX_VPANEL_DESIRED;
            }
        }
        else {
            new_Vpanel_desired = Vpanel_desired - perturbation_step_size;
            if((new_Vpanel_desired < MIN_VPANEL_DESIRED) || (new_Vpanel_desired > MAX_VPANEL_DESIRED)) {
                new_Vpanel_desired = MIN_VPANEL_DESIRED;
            }
        }
        Vpanel_desired = new_Vpanel_desired;


        PPT_count--;
        if(PPT_count == 0) {
            PPT_I_max = 0;
            PPT_count = perturbation_steps;
            PPT_direction = 3;
        }
        break;

    case 3:
        PPT_direction = 4;
        break;
    case 4:
        PPT_direction = 3;
        if(I >= PPT_I_max) {
            PPT_I_max = I;
            PPT_V_max2 = V;
        }

        new_Vpanel_desired = Vpanel_desired - perturbation_step_size;
        if((new_Vpanel_desired < MIN_VPANEL_DESIRED) || (new_Vpanel_desired > MAX_VPANEL_DESIRED)) {
            new_Vpanel_desired = MIN_VPANEL_DESIRED;
        }
        Vpanel_desired = new_Vpanel_desired;

        PPT_count--;
        if(PPT_count == 0) {
            PPT_I_max = 0;
            PPT_count = perturbation_steps;
            PPT_direction = 5;
        }
        break;

    case 5:
        PPT_direction = 6;
        break;
    case 6:
        PPT_direction = 5;
        if(I >= PPT_I_max) {
            PPT_I_max = I;
            PPT_V_max3 = V;
        }

        new_Vpanel_desired = Vpanel_desired - perturbation_step_size;
        if((new_Vpanel_desired < MIN_VPANEL_DESIRED) || (new_Vpanel_desired > MAX_VPANEL_DESIRED)) {
            new_Vpanel_desired = MIN_VPANEL_DESIRED;
        }
        Vpanel_desired = new_Vpanel_desired;

        PPT_count--;
        if(PPT_count == 0) {
            PPT_I_max = 0;
            PPT_count = perturbation_steps;
            PPT_direction = 7;
        }
        break;

    case 7:
        PPT_direction = 8;
        break;
    case 8:
        PPT_direction = 7;
        if(I >= PPT_I_max) {
            PPT_I_max = I;
            PPT_V_max4 = V;
        }

        if(I > PPT_MIN_CURRENT_THRESHOLD) {
            new_Vpanel_desired = Vpanel_desired + perturbation_step_size;
            if(new_Vpanel_desired > Vbatt_ADC - MIN_VPANEL_TO_VBATT) {
                new_Vpanel_desired = Vbatt_ADC - MIN_VPANEL_TO_VBATT;
            }
            if(new_Vpanel_desired > MAX_VPANEL_DESIRED) {
                new_Vpanel_desired = MAX_VPANEL_DESIRED;
            }
        }
        else {
            new_Vpanel_desired = Vpanel_desired - perturbation_step_size;
            if((new_Vpanel_desired < MIN_VPANEL_DESIRED) || (new_Vpanel_desired > MAX_VPANEL_DESIRED)) {
                new_Vpanel_desired = MIN_VPANEL_DESIRED;
            }
        }
        Vpanel_desired = new_Vpanel_desired;

        PPT_count--;
        if(PPT_count == 0) {
            if(I < PPT_LOW_CURRENT_THRESHOLD) {
                if(perturbation_steps > MIN_PERTURBATION_STEPS) {
                    perturbation_steps--;
                }
            }
            else {
                if(perturbation_steps < MAX_PERTURBATION_STEPS) {
                    perturbation_steps++;
                }
            }

            PPT_I_max = 0;
            PPT_count = perturbation_steps;
            PPT_direction = 1;

            PPT_V_max = (PPT_V_max1 + PPT_V_max2 + PPT_V_max3 + PPT_V_max4) >> 11;
            Vpanel_desired = PPT_V_max;

        }

        break;

    }
}

void init_dithering_algorithm(void)
{
    PPT_direction = -1;
    perturbation_steps = MIN_PERTURBATION_STEPS;
    perturbation_step_size = PERTURBATION_STEP_SIZE;
    PPT_count = perturbation_steps;
    PPT_accumulator = 0;
    PPT_I_max = 0;
}

void dithering_algorithm(uint32 I, uint32 V)
{
    int16 new_Vpanel_desired;

//  UART_queue_binary(I >> 9, 4); // DEBUG
//  UART_queue_binary(V >> 9, 4); // DEBUG
//  UART_start_transmission();    // DEBUG

    if(PPT_count < 0) {
        PPT_accumulator -= I;
    }
    else if(PPT_count > 0) {
        PPT_accumulator += I;
    }

    if(I > PPT_I_max) {
        PPT_I_max = I;
    }

    new_Vpanel_desired = Vpanel_desired;
    if(I < PPT_MIN_CURRENT_THRESHOLD) {
        new_Vpanel_desired -= perturbation_step_size;
    }

    if(PPT_direction == 1) {
        if(PPT_count == perturbation_steps) {
            PPT_direction = -1;

            if( (PPT_accumulator > 0) || (Vbattery > battery_charge_voltage) ) {
                new_Vpanel_desired += PPT_STEP_SIZE;
            }
            else {
                new_Vpanel_desired -= PPT_STEP_SIZE;
            }
            if(PPT_I_max <= PPT_LOW_CURRENT_THRESHOLD) {
                if(perturbation_steps < MAX_PERTURBATION_STEPS) {
                    perturbation_steps++;
                    PPT_count = perturbation_steps;
                }
            }
            else {
                if(perturbation_steps > MIN_PERTURBATION_STEPS) {
                    perturbation_steps--;
                    PPT_count = perturbation_steps;
                }
            }
            PPT_accumulator = 0;
            PPT_I_max = 0;

        }
        else {
            PPT_count++;
            new_Vpanel_desired += perturbation_step_size;
        }
    }
    else {
        if(PPT_count == -perturbation_steps) {
            PPT_direction = 1;
        }
        else {
            PPT_count--;
            new_Vpanel_desired -= perturbation_step_size;
            if((new_Vpanel_desired < MIN_VPANEL_DESIRED) || (new_Vpanel_desired > MAX_VPANEL_DESIRED)) {
                new_Vpanel_desired = MIN_VPANEL_DESIRED;
            }
        }
    }

    if(new_Vpanel_desired < MIN_VPANEL_DESIRED) {
        new_Vpanel_desired = MIN_VPANEL_DESIRED;
    }
    if(new_Vpanel_desired > Vbatt_ADC - MIN_VPANEL_TO_VBATT) {
        new_Vpanel_desired = Vbatt_ADC - MIN_VPANEL_TO_VBATT;
    }
    if(new_Vpanel_desired > MAX_VPANEL_DESIRED) {
        new_Vpanel_desired = MAX_VPANEL_DESIRED;
    }

    Vpanel_desired = new_Vpanel_desired;
}

void tracking_algorithm(uint32 I, uint32 V)
{
//  return; //DEBUG
    if(!T2CONbits.TON) {  // check if the converter has been shut off because of battery overvoltage
        if(tracking_mode > CHECK_CONDITIONS_TRACKING_MODE) {
            tracking_mode = INITIAL_TRACKING_MODE;
        }
    }

    switch(tracking_mode) {
    case INITIAL_TRACKING_MODE:
        init_check_power_on_conditions();
        tracking_mode++;
        break;
	case BUTTON_PRESSED_MODE:
		if( (button_delay == 0) && (power_on) ) {
	        tracking_mode++;		
		}
		break;
    case CHECK_CONDITIONS_TRACKING_MODE:  // power converter is off in this mode
        if(check_power_on_conditions()) {
            input_PWM_init();
			if(constant_voltage == 0) {
            	init_full_scan_algorithm();
            	tracking_mode = FULL_SCAN_TRACKING_MODE;
			}
			else {
				set_constant_voltage(constant_voltage);
            	tracking_mode = CONSTANT_VOLTAGE_TRACKING_MODE;
			}
        }
        break;
	case CONSTANT_VOLTAGE_TRACKING_MODE:  // the power converter maintains a constant voltage
		__asm__("bset LATB, #7");
		break;
    case FULL_SCAN_TRACKING_MODE:  // do a full scan to find the global peak power point
      	if(full_scan_algorithm(I, V)) {
           	init_dithering_algorithm();
           	tracking_mode++;
       	}
        break;
    case DITHERING_TRACKING_MODE:  // dither around the peak power point to continuously optimize it
        dithering_algorithm(I, V);
        break;
    }
}

void compute_averages(void)
{
    if(global_flags & SNAPSHOT_TAKEN) {
        Ppanel_ADC_accumulator += (Ppanel_ADC_accumulator_snapshot + (1<<2)) >> 3;
        Vpanel_ADC_accumulator += Vpanel_ADC_accumulator_snapshot;
        Ipanel_ADC_accumulator += Ipanel_ADC_accumulator_snapshot;
        Vbattery_ADC_accumulator += Vbattery_ADC_accumulator_snapshot;

        tracking_algorithm(Ipanel_ADC_accumulator_snapshot, Vpanel_ADC_accumulator_snapshot);

        global_flags &= ~SNAPSHOT_TAKEN;
        snapshot_accumulator_counter++;
        if(snapshot_accumulator_counter == 8) {
            Ipanel =   ((((Ipanel_ADC_accumulator +   (U1<<7)) >> 8) * IPANEL_CALIBRATION) +   (U1<<19)) >> 20;
            Vpanel =   ((((Vpanel_ADC_accumulator +   (U1<<7)) >> 8) * VPANEL_CALIBRATION) +   (U1<<19)) >> 20;
            Vbattery = ((((Vbattery_ADC_accumulator + (U1<<7)) >> 8) * VBATTERY_CALIBRATION) + (U1<<19)) >> 20;
            Ppanel = ((((Ppanel_ADC_accumulator + (U1<<15)) >> 16) * PPANEL_CALIBRATION) + (U1<<15)) >> 16;

//          tracking_algorithm(Ipanel);

            Vpanel_ADC_accumulator = 0;
            Ipanel_ADC_accumulator = 0;
            Ppanel_ADC_accumulator = 0;
            Vbattery_ADC_accumulator = 0;
            snapshot_accumulator_counter = 0;
        }
    }
}

void init_variables(void)
{
    Ipanel = 0;
    Vpanel = 0;
    Ppanel = 0;
    Vbattery = 0;
    Ipanel_ADC_accumulator = 0;
    Vpanel_ADC_accumulator = 0;
    Ppanel_ADC_accumulator = 0;
    snapshot_accumulator_counter = 0;
    global_flags = 0;
    tracking_mode = 0;
	constant_voltage = 0;
    send_sequence = 0;
}

int main(void)
{
    LATA = 0;
    LATB = 0;
    TRISA = 0b0000000000001111;
    TRISB = 0b1000011000000111;
//    LATBbits.LATB7 = 1; // Turn on Status LED

    set_up_clock();
	init_variables();
	battery_charge_voltage = read_battery_charge_voltage();
	ppt_node_id = read_ppt_node_id();
    push_button_init();
    UART_init();
    CAN_init(ppt_node_id);

// DEBUG CODE
/*	update_ppt_node_id(0);
	while(1) {
		if((CAN_command = CAN_receive_command()) != -1) {
			if(CAN_command == 2) {
				if(CAN_get_payload_voltage_ADC() == 300) {
					__asm__("btg LATB, #7");
				}
			}
		}
	} */
// END DEBUG CODE

    ADC_init();
    input_capture_init();
    timekeeper_init();

    while(1) {
        int16 event = wait_for_timekeeper_event();

        if(event & (1 << 4)) {
            UART_receive();
            UART_send();
//          UART_process();  // DEBUG
            compute_averages();
        }

        if(event & (1 << 10)) {
            check_push_button();
		}

        if(event & (1 << 11)) {

            switch(send_sequence++) {
            case 0:
                UART_send_number(Vpanel, 1);
                break;
            case 1:
                UART_send_number(Ipanel, 3);
                break;
            case 2:
                UART_send_number(Ppanel, 1);
                break;
            case 3:
                UART_send_number(Vbattery, 1);
                break;
            case 4:
                UART_send_number(Iavg, 0);
                break;
            case 5:
                UART_send_number(Vpanel_desired, 0);
                break;
            case 6:
                UART_send_number(Vpanel_ADC, 0);
                break;
            case 7:
                UART_send_number(integral_error, 0);
                break;
            case 8:
                UART_send_number(LSadjust, 0);
                break;
            case 9:
                UART_send_number(Imax_found, 0);
                break;
            case 10:
                UART_send_divider();
                send_sequence = 0;
                break;
            }

			if((CAN_command = CAN_receive_command()) != -1) {
				switch(CAN_command) {
				case CAN_COMMAND_POWER_OFF:
			        input_PWM_disable();
					tracking_mode = INITIAL_TRACKING_MODE;
					power_on = 0;
					__asm__("bclr LATB, #7");
					break;
				case CAN_COMMAND_FULL_SCAN:
			        input_PWM_disable();
					tracking_mode = INITIAL_TRACKING_MODE;
					power_on = 1;
					constant_voltage = 0;
					break;
				case CAN_COMMAND_CONSTANT_VOLTAGE:
					constant_voltage = CAN_get_payload_voltage_ADC();
					if( (tracking_mode == CONSTANT_VOLTAGE_TRACKING_MODE) ||
						(tracking_mode == FULL_SCAN_TRACKING_MODE) ||
						(tracking_mode == DITHERING_TRACKING_MODE) ) {
           				tracking_mode = CONSTANT_VOLTAGE_TRACKING_MODE;
						set_constant_voltage(constant_voltage);
					}
					else {
           				tracking_mode = INITIAL_TRACKING_MODE;
						power_on = 1;
					}
					break;
				case CAN_COMMAND_SET_BATTERY_CHARGE_VOLTAGE:
			        input_PWM_disable();
					tracking_mode = INITIAL_TRACKING_MODE;
					battery_charge_voltage = CAN_get_payload_voltage();
					write_data_to_flash(ppt_node_id, battery_charge_voltage);
					break;
				}
			}
		}

        if(event & (1 << 13)) {
            if(!PORTBbits.RB9) {   // Only send data to the CAN if the 12V power supply to the CAN is present
                CAN_send(Vpanel, Ppanel, Vbattery, Ipanel); // !!! NOTE THAT Ipanel is actually the BATTERY current... !!!
            }
		}

        if(event & (1 << 14)) {
            if(!PORTBbits.RB9) {   // Only send data to the CAN if the 12V power supply to the CAN is present
                CAN_send_status(tracking_mode);
            }
		}

        if(event & (1 << 10)) {
            if(tracking_mode == FULL_SCAN_TRACKING_MODE) {
                __asm__("btg LATB, #7");
            }
        }

        if(event & (1 << 14)) {
            if(tracking_mode == CHECK_CONDITIONS_TRACKING_MODE) {
                __asm__("btg LATB, #7");
            }
        }

        if(event & (1 << 11)) {
            if(tracking_mode == DITHERING_TRACKING_MODE) {
                if(LED_dither_counter <= 0) {

                }
                else if(LED_dither_counter <= 5) {
                    __asm__("btg LATB, #7");
                }
                else {
                    __asm__("bclr LATB, #7");
                    LED_dither_counter = -5;
                }
                LED_dither_counter += 1;
            }
        }
    }
    return 0;
}



