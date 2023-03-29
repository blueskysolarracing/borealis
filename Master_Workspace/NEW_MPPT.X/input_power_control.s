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

.title "Input Power Control"

.include "p24HJ64GP502.inc"

; input power section parameters
.equ INDUCTOR_CONSTANT, 850;1306/90uH,1082/70uH?,850/68uH
.equ LH_DEADTIME, 25
.equ HL_DEADTIME, 5
.equ MAX_LS_OFF, 1400				
.equ MIN_HS_OFF, 42 ;the moment ls turns off
.equ MAX_HS_OFF, 1400
.equ MIN_LS_ON_TIME, 1 ; must be 1 or more
.equ IDEAL_CAPTURE_DELAY, 20 ;23 for coilcraft inductor, ideal time between HS off and trigger in clock cycles //85uH: 12/4.5us, 17 for 90uH, 25 for 100uH
.equ MAX_LS_ADJUST_DELTA, 250
.equ NOMINAL_LS_ADJUST, 32768  
.equ MIN_LS_ADJUST, (NOMINAL_LS_ADJUST / 2)
.equ MAX_LS_ADJUST, (40000)
.equ MAX_UPDATE_TIME_2, 70
.equ MAX_IAVG, 5500
				    

.equ MAX_VBATT, (150 * 20)  ; This value must be the same as in controller1.c ; set to 140V
.equ MIN_VBATT, (50 * 20)  ; This value must be the same as in controller1.c ; set to 90V
.equ MIN_VPANEL_TO_VBATT, (1 * 20) ; This value must be the same as in controller1.c ; set to 15V
; Must: MIN_VPANEL_TO_VBATT <= MIN_VBATT
.equ MIN_VPANEL, (5 * 20)  ; This value must be the same as in controller1.c ; set to 10V

; DEBUG START
;.equ MIN_VBATT, (1 * 20)
;.equ MIN_VPANEL_TO_VBATT, (1 * 20)
;.equ MIN_VPANEL, (1 * 20)
; DEBUG END

.equ INITIAL_VPANEL_DESIRED, 200 * 20 ; use high value to force zero current
.equ PROPORTIONAL_SCALAR, 1000
.equ INTEGRAL_SCALAR, 1
.equ DIFFERENTIAL_SCALAR, 2000
.equ MAX_INTEGRAL_ERROR, 5000
.equ MIN_INTEGRAL_ERROR, -5000

.bss
w15_saved:          .space 2
Vcharge_ADC:        .space 2
Vdischarge_ADC:     .space 2
; flags[0] is set when we are in low current / extended period mode
flags:              .space 2
.global _Vpanel_desired
_Vpanel_desired:    .space 2
integral_error:     .space 2
.global _integral_error
_integral_error:    .space 0
integral_errorH:    .space 2
old_Vout_error:     .space 2
.global _Iavg
_Iavg:              .space 0
Iavg:               .space 2
.global _LSadjust
_LSadjust:          .space 0
LSadjust:           .space 2
capture_delay:      .space 2
power_accumulatorL: .space 2
power_accumulatorH: .space 2
power_counter:      .space 2
power_delta_accumulatorL: .space 2
power_delta_accumulatorH: .space 2
power_delta_accumulatorU: .space 2

.global _Ipanel_ADC
_Ipanel_ADC: .space 0
Ipanel_ADC: .space 2
.global _Vpanel_ADC
_Vpanel_ADC: .space 0
Vpanel_ADC: .space 2
.global _Vbatt_ADC
_Vbatt_ADC: .space 0
Vbatt_ADC: .space 2

ADC_accumulator_counter: .space 2
Ipanel_ADC_accumulatorL: .space 2
Ipanel_ADC_accumulatorH: .space 2
Vpanel_ADC_accumulatorL: .space 2
Vpanel_ADC_accumulatorH: .space 2
Ppanel_ADC_accumulatorL: .space 2
Ppanel_ADC_accumulatorH: .space 2
Vbattery_ADC_accumulatorL: .space 2
Vbattery_ADC_accumulatorH: .space 2

.equ SNAPSHOT_TAKEN, 0
.global _global_flags
.global _Ipanel_ADC_accumulator_snapshot
_Ipanel_ADC_accumulator_snapshot:  .space 2
_Ipanel_ADC_accumulator_snapshotH: .space 2
.global _Vpanel_ADC_accumulator_snapshot
_Vpanel_ADC_accumulator_snapshot:  .space 2
_Vpanel_ADC_accumulator_snapshotH: .space 2
.global _Ppanel_ADC_accumulator_snapshot
_Ppanel_ADC_accumulator_snapshot:  .space 2
_Ppanel_ADC_accumulator_snapshotH: .space 2
.global _Vbattery_ADC_accumulator_snapshot
_Vbattery_ADC_accumulator_snapshot:  .space 2
_Vbattery_ADC_accumulator_snapshotH: .space 2

; timing information for the input power section follows:
.equ HS_ON_OFFSET, -2
.equ HS_OFF_OFFSET, -4
.equ LS_ON_OFFSET, -6
.equ LS_OFF_OFFSET, -8
.equ PERIOD_OFFSET, -10
; pointer to the timing structure which will be used by the interrupt routine
; to update all timing values
timing_info_pointer:   .space 2
; first timing info structure
period1:        .space 2
LSoff1:         .space 2
LSon1:          .space 2
HSoff1:         .space 2
HSon1:          .space 2
timing_info1:   .space 0
; second timing info structure
period2:        .space 2
LSoff2:         .space 2
LSon2:          .space 2
HSoff2:         .space 2
HSon2:          .space 2
timing_info2:   .space 0
; capture information 
capture_info_start: .space 0
capture_ref:   .space 2
period_ref:    .space 2
ICxCON:        .space 2
ICxBUF:        .space 2
; end of timing info structures

.text

;===============================================================================
clear_accumulators:
clr Vpanel_ADC_accumulatorL
clr Vpanel_ADC_accumulatorH
clr Ipanel_ADC_accumulatorL
clr Ipanel_ADC_accumulatorH
clr Ppanel_ADC_accumulatorL
clr Ppanel_ADC_accumulatorH
clr Vbattery_ADC_accumulatorL
clr Vbattery_ADC_accumulatorH
return

;===============================================================================
.global compute_input_power
.global _compute_input_power
compute_input_power:  ; Compute the power of the input stage (panel to battery)
_compute_input_power:
; Input: w7 is the timing flags
;-------------------------------------------------------------------------------

; get the current and average into the accumulator
mov Ipanel_ADC, w0
add Ipanel_ADC_accumulatorL
clr w0
addc Ipanel_ADC_accumulatorH

; get the solar panel voltage and average into the accumulator
mov Vpanel_ADC, w0
add Vpanel_ADC_accumulatorL
clr w0
addc Vpanel_ADC_accumulatorH

; get the battery voltage and average into the accumulator
mov Vbatt_ADC, w0
add Vbattery_ADC_accumulatorL
clr w0
addc Vbattery_ADC_accumulatorH

; compute the power and average into the accumulator
mov Ipanel_ADC, w0
lsr w0, #1, w0
mov Vbatt_ADC, w1
mul.uu w0, w1, w0 ; power is in w1:w0
add Ppanel_ADC_accumulatorL
mov w1, w0
addc Ppanel_ADC_accumulatorH

btss w7, #8 ; this will cause 512 values to be accumulated, which is the maximum allowed, otherwise we risk overflowing the accumulator
bra 1f
mov Ipanel_ADC_accumulatorL, w0
mov w0, _Ipanel_ADC_accumulator_snapshot
mov Ipanel_ADC_accumulatorH, w0
mov w0, _Ipanel_ADC_accumulator_snapshotH
mov Vpanel_ADC_accumulatorL, w0
mov w0, _Vpanel_ADC_accumulator_snapshot
mov Vpanel_ADC_accumulatorH, w0
mov w0, _Vpanel_ADC_accumulator_snapshotH
mov Vbattery_ADC_accumulatorL, w0
mov w0, _Vbattery_ADC_accumulator_snapshot
mov Vbattery_ADC_accumulatorH, w0
mov w0, _Vbattery_ADC_accumulator_snapshotH
mov Ppanel_ADC_accumulatorL, w0
mov w0, _Ppanel_ADC_accumulator_snapshot
mov Ppanel_ADC_accumulatorH, w0
mov w0, _Ppanel_ADC_accumulator_snapshotH
bset _global_flags, #SNAPSHOT_TAKEN
call clear_accumulators
1:

return

;===============================================================================
input_PWM_timing_init:
mov #INITIAL_VPANEL_DESIRED, w0
mov w0, _Vpanel_desired
clr integral_error
clr integral_errorH
clr old_Vout_error
clr Iavg
mov #NOMINAL_LS_ADJUST, w0
mov w0, LSadjust
bset ICxCON, #15  ; nothing captured right now, set flag
clr HSon1
clr HSon2 
mov #timing_info2, w0
mov w0, timing_info_pointer
return

;===============================================================================
.global _input_PWM_init
_input_PWM_init:
; Input: none
; Changes: w0
;-------------------------------------------------------------------------------

bclr LATB, #12  ; when the OC module is disabled, we want the pins driven low
bclr TRISB, #12
bclr LATB, #13
bclr TRISB, #13

bset LATB, #12 ; brief pulse on LS switch to charge the HS gate driver cap
call input_PWM_timing_init
bclr LATB, #12
call compute_input_PWM_timing
call clear_accumulators

bclr IFS0, #T2IF
bset IPC1, #T2IP1 ; set the TMR2 priority to 7, the highest possible
bset IPC1, #T2IP0
bset IEC0, #T2IE
bset IFS0, #T2IF ; trigger the interrupt to set up the PWM timing

mov #-100, w0
mov w0, TMR2
bset T2CON, #TON ; start timer 2 using default configuration

; inductor charge switch
mov #0b0000000000000101, w0  ; dual compare mode, continuous pulses, using TMR2
mov w0, OC1CON

; inductor discharge switch
mov #0b0000000000000101, w0  ; dual compare mode, continuous pulses, using TMR2
mov w0, OC2CON

mov #(18), w0      ; assign OC1 module (18)
ior RPOR6         ; to pin RP12 -> drives the inductor charge switch
mov #(19 << 8), w0 ; assign OC2 module (19)
ior RPOR6         ; to pin RP13 -> drives the inductor discharge switch

return

;===============================================================================
.global _input_PWM_disable
_input_PWM_disable:

clr OC1CON
clr OC2CON
bclr T2CON, #TON ; turn off timer 2

return

;===============================================================================
.global	__T2Interrupt
__T2Interrupt:
mov w15, w15_saved
btss IEC0, #T2IE
bra 1f
mov #capture_info_start, w15
push OC2RS
push PR2
push IC1CON
push IC1BUF
mov timing_info_pointer, w15
pop OC1R
pop OC1RS
pop OC2R
pop OC2RS
pop PR2
mov IC1BUF, w15 ; in case there are other captured events, clear them all
mov IC1BUF, w15 ; the fifo buffer holds up to four values so we need three
mov IC1BUF, w15 ; more reads to be sure we have cleared everything

bclr IEC0, #T2IE ; disable the interrupt until new timing info is calculated
1:
bclr IFS0, #T2IF ; clear the flag that caused this interrupt
mov w15_saved, w15
retfie

;===============================================================================
.global compute_input_PWM_timing
.global _compute_input_PWM_timing
compute_input_PWM_timing:
_compute_input_PWM_timing:
push w14

; compute the inductor charge voltage (in ADC units), make sure it is 1 or greater
mov Vpanel_ADC, w0
cp w0, #0
; if statement
bra NZ, 1f
mov #1, w0
1:
mov w0, Vcharge_ADC

; Compute the inductor discharge voltage (in ADC units), make sure it is 1 or greater
mov Vbatt_ADC, w0
subr Vpanel_ADC, WREG
bra GT, 1f
mov #1, w0
1:
mov w0, Vdischarge_ADC

; If battery voltage too high or too low, quickly disable power transfer
mov #MAX_VBATT, w0
sub Vbatt_ADC, WREG
bra N, 1f
call _input_PWM_disable
bra exit_interrupt_routine
1:
mov #MIN_VBATT, w0
sub Vbatt_ADC, WREG
bra NN, 1f
call _input_PWM_disable
bra exit_interrupt_routine
1:
; If the panel voltage is too low, then disable power transfer
mov #MIN_VPANEL, w0
sub Vpanel_ADC, WREG
bra NN, 1f
call _input_PWM_disable
bra exit_interrupt_routine
1:

; if the desired voltage is too close to the battery voltage, then lower the desired voltage
mov #MIN_VPANEL_TO_VBATT, w0
sub Vbatt_ADC, WREG
mov w0, w1
subr _Vpanel_desired, WREG
bra NN, 1f
mov w1, _Vpanel_desired
1:

; PID controller section ==================================================================
mov _Vpanel_desired, w1
mov Vpanel_ADC, w0
sub w1, w0, w2  ; w2 now contains the error in the output voltage

; Integral error section

mov #INTEGRAL_SCALAR, w1
mul.us w1, w2, w0
sub integral_error
mov w1, w0
subb integral_errorH

mov #MAX_INTEGRAL_ERROR, w0
cp integral_errorH
bra LE, 1f
mov w0, integral_errorH
clr integral_error
bra 2f
1:
mov #MIN_INTEGRAL_ERROR, w0
cp integral_errorH
bra GE, 2f
mov w0, integral_errorH
clr integral_error
2:


;mov integral_error, w1
;sub w1, w2, w0

;mov #MAX_INTEGRAL_ERROR, w1
;cp w0, w1
;bra LE, 1f
;mov w1, w0
;bra 2f
;1:
;mov #MIN_INTEGRAL_ERROR, w1
;cp w0, w1
;bra GE, 2f
;mov w1, w0
;2:
;mov w0, integral_error

mov integral_error, w4
mov integral_errorH, w5

; Differential error section

mov old_Vout_error, w1
mov w2, old_Vout_error
sub w2, w1, w0
mov #DIFFERENTIAL_SCALAR, w1
mul.us w1, w0, w0
sub w4, w0, w4
subb w5, w1, w5

; Proporional error section

mov #PROPORTIONAL_SCALAR, w1
mul.us w1, w2, w0
sub w4, w0, w4
subb w5, w1, w5

; Make sure our computed Iavg is within limits
cp w5, #0
bra NN, 1f
clr Iavg
bra 2f
1:
mov #MAX_IAVG, w3
mov #0b1111111000000000, w0
and w5, w0, w0
bra NZ, 1f
sl w5, #7, w5
lsr w4, #9, w4
ior w4, w5, w5
cp w5, w3
bra GTU, 1f
mov w5, w3
1:

; Save the computed Iavg
mov w3, Iavg
2:

;.global _PID  ; DEBUG
;_PID:         ; DEBUG

;mov #1000, w0 ; DEBUG
;mov _Vpanel_desired, w5  ; DEBUG
;sub w0, w5, w5 ; DEBUG
;mov w5, Iavg   ; DEBUG
;bra NN, 1f     ; DEBUG
;clr Iavg       ; DEBUG
;1:             ; DEBUG

;mov #5000, w0  ; DEBUG
;mov w0, Iavg   ; DEBUG

; End of PID controller section ==========================================================

; clear the flags
clr flags

; get the pointer to the location where we will store timing parameters, w14 will be the pointer
mov #timing_info1, w14
mov timing_info_pointer, w0
cpsne w0, w14
mov #timing_info2, w14

; compute the capture delay error if a capture value is available
btsc ICxCON, #15
bra no_capture_available
1:
bset ICxCON, #15
mov capture_ref, w0
mov period_ref, w1
mov ICxBUF, w2
mov ICxCON, w3
btss w3, #15
bra 1b
btss w3, #ICBNE
bra no_capture_available
cp w2, w0
bra GEU, 1f
sub w1, w0, w0
add w2, w0, w0
bra 2f
1:
sub w2, w0, w0
2:
; at this point, w0 contains the capture_delay
mov w0, capture_delay
sub #IDEAL_CAPTURE_DELAY, w0  ; w0 now contains the capture_delay_error

mov #MAX_LS_ADJUST_DELTA, w2  ; make sure LSadjust_delta is in an acceptable range
cp w0, w2
bra LE, 1f
mov w2, w0
1:
mov #(-MAX_LS_ADJUST_DELTA), w2
cp w0, w2
bra GE, 1f
mov w2, w0
1:
; w0 now contains LSadjust_delta that's been clamped to a (min, max) range

sl w0, #6, w0  ; amplify the LSadjust_delta

add LSadjust, WREG
mov #MAX_LS_ADJUST, w2  ; make sure LSadjust is in an acceptable range
cp w0, w2
bra LEU, 1f
mov w2, w0
1:
mov #(MIN_LS_ADJUST), w2
cp w0, w2
bra GEU, 1f
mov w2, w0
1:
; w0 now contains the new LSadjust that's been clamped to a (min, max) range
mov w0, LSadjust

no_capture_available:

; Calculation of the new HSoff = 2*L*Iavg / Vcharge
mov #INDUCTOR_CONSTANT, w0
mov Iavg, w1
mul.uu w0, w1, w2
lsr w2, #4, w2
sl w3, #12, w4
ior w2, w4, w2
lsr w3, #4, w3
mov #MAX_HS_OFF, w5
mov Vcharge_ADC, w4
repeat #17
div.ud w2, w4
bra OV, 1f
cp w0, w5
bra GTU, 1f
mov w0, w5
1:
mov #MIN_HS_OFF, w0
cp w5, w0
bra GEU, 1f
mov w0, w5
bset flags, #0
1:
mov w5, [w14 + HS_OFF_OFFSET]
add w5, #HL_DEADTIME, w6
mov w6, [w14 + LS_ON_OFFSET]

; Compute LSoff
mov #MAX_LS_OFF, w7  ; although this sounds like the maximum, in fact,
                     ; LSoff can end up being 1.5 times this value when it
                     ; gets multiplied below.
mov Vcharge_ADC, w0
mul.uu w5, w0, w0
mov Vdischarge_ADC, w2
repeat #17
div.ud w0, w2
bra OV, 1f
cp w0, w7
bra GTU, 1f
mov w0, w7
1:
mov LSadjust, w1
mul.uu w7, w1, w0
rlc w0, w0
rlc w1, w1
add w5, w1, w7
mov w7, [w14 + LS_OFF_OFFSET]

; Compute the period
btss flags, #0
bra done_computing_period

mov #65535, w7  ; By default, the period is the maximum possible


mov [w14 + LS_OFF_OFFSET], w0
mov #MIN_HS_OFF, w1
mov Vcharge_ADC, w2
sl w2, #4, w2
; need to multiply w0, w1, w2
mul.uu w0, w1, w0
mul.uu w1, w2, w4
mul.uu w0, w2, w0
add w1, w4, w1
addc w5, #0, w2
; numerator now in w2:w1:w0



;mov [w14 + LS_OFF_OFFSET], w0
;mov w0, w1
;sub #MIN_HS_OFF, w1
;mov Vdischarge_ADC, w2
;sl w2, #4, w2
; need to multiply w0, w1, w2
;mul.uu w0, w1, w0
;mul.uu w1, w2, w4
;mul.uu w0, w2, w0
;add w1, w4, w1
;addc w5, #0, w2
; numerator now in w2:w1:w0

mov #INDUCTOR_CONSTANT, w4
mov Iavg, w5
mul.uu w4, w5, w4
; denominator is in w5:w4

cp w5, #0
bra Z, do_division_now

ff1l w5, w3
dec w3, w3
subr w3, #16, w6

lsr w4, w3, w4
sl w5, w6, w5
ior w4, w5, w4

lsr w0, w3, w0
mov w1, w5
sl w5, w6, w5
ior w5, w0, w0
lsr w1, w3, w1
mov w2, w5
sl w5, w6, w5
ior w5, w1, w1
lsr w2, w3, w2

do_division_now:
cp w2, #0
bra NZ, done_computing_period
cp w4, #0
bra Z, done_computing_period

repeat #17
div.ud w0, w4
bra OV, done_computing_period
add w0, #LH_DEADTIME, w0
bra C, done_computing_period
mov w0, w7

done_computing_period:
mov w7, [w14 + PERIOD_OFFSET]

; Now check that the timing parameters are ok with respect to each other,
; adjust if necessary
mov [w14 + LS_OFF_OFFSET], w0
mov [w14 + LS_ON_OFFSET], w1
mov [w14 + PERIOD_OFFSET], w2
add w1, #MIN_LS_ON_TIME, w1
cp w0, w1
bra GEU, 1f
mov w1, w0
1:
sub w2, #LH_DEADTIME, w3
cp w3, w0
bra GEU, 1f
add w0, #LH_DEADTIME, w2
bra NC, 1f
mov #65535, w2
sub w2, #LH_DEADTIME, w0
1:
mov w0, [w14 + LS_OFF_OFFSET]
mov w2, [w14 + PERIOD_OFFSET]

; Now, activate the new timing information
mov w14, timing_info_pointer ; timing data is ready, next update of the PWM will use it
bclr IFS0, #T2IF ; clear the flag so we don't get a premature interrupt
bset IEC0, #T2IE ; enable the interrupt so that the new timing info can take effect

; Determine how much time is left before the end of this period
; If there is time, immediately update timing information
mov PR2, w0
mov TMR2, w1
cp PR2
bra NZ, exit_interrupt_routine ; check that an interrupt didn't occur in the time between
                               ; reading PR3 and reading TMR3 

sub w0, w1, w0
mov #MAX_UPDATE_TIME_2, w3
cp w0, w3
bra LTU, exit_interrupt_routine
add w1, w3, w2
mov OC1RS, w0
cp w2, w0
bra GEU, update_period_only
mov [w14 + HS_OFF_OFFSET], w0
cp w0, w2
bra LEU, update_period_only
mov w0, OC1RS
mov [w14 + LS_ON_OFFSET], w0
mov w0, OC2R
mov [w14 + LS_OFF_OFFSET], w0
mov w0, OC2RS
mov [w14 + PERIOD_OFFSET], w0
mov w0, PR2
bra exit_interrupt_routine
update_period_only:
mov OC2RS, w0
add w0, #LH_DEADTIME, w0
bra C, exit_interrupt_routine
cp w0, w2
bra LEU, 1f
mov w0, w2
1:
mov [w14 + PERIOD_OFFSET], w0
cp w0, w2
bra GEU, 1f
mov w2, w0
1:
mov w0, PR2
exit_interrupt_routine:

pop w14
return

.end



