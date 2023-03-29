#include <p24hj64gp502.h>
#include "types.h"

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)
// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)
// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
//#pragma config GCP = OFF
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)
// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)
// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD3               // Comm Channel Select (Communicate on PGC3/EMUC3 and PGD3/EMUD3)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

#define VPANEL_CALIBRATION 32478
#define VBATTERY_CALIBRATION 34862
#define IPANEL_CALIBRATION 60357
#define PPANEL_CALIBRATION ((uint16)((uint32)VBATTERY_CALIBRATION/2*IPANEL_CALIBRATION/128000))

extern void set_up_clock();
extern void ADC_init();
extern void input_capture_init(void);
extern void input_PWM_init(void);
extern void input_PWM_disable(void);
extern void timekeeper_init(void);
extern int16 wait_for_timekeeper_event();
extern void write_data_to_flash(uint16 ppt_node_id, uint16 battery_charge_voltage);
extern uint16 read_battery_charge_voltage(void);

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
	return 1;  // DEBUG
    
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
  //return; //DEBUG
  
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
            // Averaging 8 data values
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
}

int main(void)
{
    LATA = 0;
    LATB = 0;
    TRISA = 0b0000000000001111;
    TRISB = 0b1000011000000111;

    set_up_clock();
	init_variables();
	battery_charge_voltage = read_battery_charge_voltage();
    push_button_init();

    ADC_init();
    input_capture_init();
    timekeeper_init();
    while(1) {
        int16 event = wait_for_timekeeper_event();
        
        if(event & (1 << 4)) {
            compute_averages();
        }
        
        if(event & (1 << 10)) {
            check_push_button();
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