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

.title "Crystal clock routines"

.include "p24HJ64GP502.inc"

.global _set_up_clock

.global	__OscillatorFail

.bss
timeout_counter:    .space 2
crystal_start_fail: .space 2

.text

; this routine will switch the clock source to the crsytal for
; better timing accuracy, but if the crystal fails to start then
; it will switch back to the less accurate internal oscillator.
_set_up_clock:
mov #(64 - 2), w0 ; Adjust the PLL multiplier to get a Fosc of 32MHz (16 MIPS) with the 4MHz crystal
mov w0, PLLFBD
; change to primary oscillator, XT mode, with PLL
mov #0b011, w0
call clock_switch
call wait_with_timeout
btss crystal_start_fail, #0
return
;
; the crystal didn't start so we revert back to the fast RC with PLL
call change_to_internal_oscillator
return


clock_switch:
;OSCCONH (high byte) Unlock Sequence
MOV #OSCCONH, w1
MOV #0x78, w2
MOV #0x9A, w3
MOV.B w2, [w1] ; Write 0x0078
MOV.B w3, [w1] ; Write 0x009A
;Set New Oscillator Selection
MOV.B w0, [w1]
; Place 0x01 in W0 for setting clock switch enabled bit
MOV #0x01, w0
;OSCCONL (low byte) Unlock Sequence
MOV #OSCCONL, w1
MOV #0x46, w2
MOV #0x57, w3
MOV.B w2, [w1] ; Write 0x0046
MOV.B w3, [w1] ; Write 0x0057
; Enable Clock Switch
MOV.B w0, [w1] ; Request Clock Switching by Setting OSWEN bit
return

change_to_internal_oscillator:
bclr OSCCON, #OSWEN ; first, abort the oscillator switch attempt
bset CLKDIV, #8 ; change the FRCDIV to 0b001 so that the fast RC is divided by 2, giving a nominal 3.685MHz
mov #(70 - 2), w0 ; Adjust the PLL multiplier to get a Fosc of about 32MHz (16 MIPS)
mov w0, PLLFBD
mov #(63 - 2), w0 ; Tune the FRC oscillator in order to get closer to a perfect 32 MHz after PLL
mov w0, OSCTUN
mov #0b001, w0 ; switch to the fast RC with PLL
call clock_switch
call wait_with_timeout
return

; the timeout is about 90ms, then we give up waiting for the oscillator switch, assume the crystal failed. 
wait_with_timeout:
clr crystal_start_fail
mov #65535, w0
mov w0, timeout_counter
next:
btss OSCCONL, #OSWEN
return
dec timeout_counter
bra NZ, next
bset crystal_start_fail, #0
return

; If the crystal fails at some random time, then the fail-safe clock monitor causes a trap
; and runs this routin. The clock source is the fast RC without PLL
__OscillatorFail:
call change_to_internal_oscillator
; clear the flags so we don't re-enter this trap
bclr OSCCON, #CF
bclr INTCON1, #OSCFAIL
retfie
