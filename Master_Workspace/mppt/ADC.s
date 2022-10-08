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

.title "ADC"

.include "p24HJ64GP502.inc"

.bss

.section ADCbuff_section, bss, dma
.global ADCbuff
ADCbuff: .space 24

.text

;===============================================================================
.global _ADC_init
_ADC_init:
; Configure the analog to digital converter
; Input: none
; Changes: w0
;-------------------------------------------------------------------------------
; set up DMA
mov #0b0000000000000000, w0
mov w0, DMA0CON
mov #0b0000000000001101, w0
mov w0, DMA0REQ
mov #dmaoffset(ADCbuff), w0
mov w0, DMA0STA
mov #0x300, w0
mov w0, DMA0PAD
mov #11, w0
mov w0, DMA0CNT
bset DMA0CON, #CHEN

; set up ADC
mov #0b0001111111110000, w0
mov w0, AD1PCFGL
mov #0b0001010011100100, w0
mov w0, AD1CON1
mov #0b0010010000001000, w0
mov w0, AD1CON2
mov #0b0000010100000001, w0 ; sample time is 5 intrucion cycles
mov w0, AD1CON3
mov #0b0000000000000010, w0 ; 4 words of buffer per analog input
mov w0, AD1CON4
mov #0b0000000000000001, w0
mov w0, AD1CHS0
mov #0b0000000000001110, w0
mov w0, AD1CSSL
bset AD1CON1, #ADON

mov #0xFFFF, w0 ; cause a delay here to let the ADC collect some samples
repeat w0       ; we will need accurate voltage measurements when we
nop             ; set the MOSFET timing

return

; get the panel current, panel voltage, and battery voltage
.global take_all_ADC_readings
take_all_ADC_readings:

mov #ADCbuff, w0
mov [w0++], w1
mov [w0++], w2
mov [w0++], w3
add w1,[w0++], w1
add w2,[w0++], w2
add w3,[w0++], w3
add w1,[w0++], w1
add w2,[w0++], w2
add w3,[w0++], w3
add w1,[w0++], w1
add w2,[w0++], w2
add w3,[w0++], w3
lsr w1, #2, w1
lsr w2, #2, w2
lsr w3, #2, w3
mov w1, _Ipanel_ADC
mov w2, _Vpanel_ADC
mov w3, _Vbatt_ADC

return

.end
