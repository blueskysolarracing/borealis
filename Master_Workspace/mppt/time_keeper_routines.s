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

.title "Timekeeper routines"

.include "p24HJ64GP502.inc"

.global _timekeeper_init
.global _wait_for_timekeeper_event

.global	__T4Interrupt

.global _global_flags

.bss
; variables
timekeeper_count:  .space 2
timekeeper_flags:  .space 2

.text
;===============================================================================
_timekeeper_init:
clr timekeeper_count
clr timekeeper_flags
clr TMR4
mov #487, w0 ; set the timer frequency to 32787 Hz, hence our bit-14 event occurs at almost exactly 1 second.
mov w0, PR4

bclr IFS1, #T4IF
bclr IPC6, #T4IP2  ; set the priority of our interrupt to 1
bset IPC6, #T4IP0
bset IEC1, #T4IE

bset T4CON, #TON ; start timer 4 using default configuration 

return

;===============================================================================
__T4Interrupt:
push w0
push w1
push w2
push w3
push w4
push w5
push w6
push w7
push RCOUNT

call take_all_ADC_readings

btsc T2CON, #TON  ; if timer 2 is off, it means that the input power section is disabled
call compute_input_PWM_timing  ; this is the main routine for controlling the MOSFET timing of the power section

mov timekeeper_count, w0
inc timekeeper_count
xor timekeeper_count, WREG
and timekeeper_count, WREG
ior timekeeper_flags

mov w0, w7
call compute_input_power  ; w7 is passed in as a parameter

pop RCOUNT
pop w7
pop w6
pop w5
pop w4
pop w3
pop w2
pop w1
pop w0
bclr IFS1, #T4IF
retfie

;                             5432109876543210
.equ TIMEKEEPER_FLAGS_MASK, 0b0110110000010000

;===============================================================================
_wait_for_timekeeper_event:
back_to_sleep:
;bclr LATA, #4  ; heartbeat LED
pwrsav #IDLE_MODE  ; no work to be done right now, so save some power
;bset LATA, #4  ; heartbeat LED
mov #TIMEKEEPER_FLAGS_MASK, w0
and timekeeper_flags   ; clear all unused flags
cp0 timekeeper_flags
bra Z, back_to_sleep
mov timekeeper_flags, w0
clr timekeeper_flags
return
