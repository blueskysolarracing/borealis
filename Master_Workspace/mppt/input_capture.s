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

.title "Input Capture"

.include "p24HJ64GP502.inc"

.global _input_capture_init

.text

;===============================================================================
_input_capture_init:
;-------------------------------------------------------------------------------
; assign pins
mov #15, w0 ; assign pin RP15 to the IC1 module
mov w0, RPINR7

; Input stage config
mov #0b0000000010000010, w0 ; capture every falling edge, capture TMR2
mov w0, IC1CON

return

.end
