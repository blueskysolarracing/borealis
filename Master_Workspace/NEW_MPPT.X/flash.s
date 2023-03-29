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

.title "FLASH"

.include "p24HJ64GP502.inc"

.equ DESTINATION_ADDRESS_NODE_ID_H, 0
.equ DESTINATION_ADDRESS_NODE_ID_L, 0xA000
.equ DESTINATION_ADDRESS_BATTERY_CHARGE_VOLTAGE_H, 0
.equ DESTINATION_ADDRESS_BATTERY_CHARGE_VOLTAGE_L, 0xA002

.bss

node_id_to_write: .space 2
battery_charge_voltage_to_write: .space 2

.text

;===============================================================================
.global _write_data_to_flash
_write_data_to_flash:
mov w0, node_id_to_write
mov w1, battery_charge_voltage_to_write

; first, erase the block
mov #0x4042, w0
mov w0, NVMCON

mov #DESTINATION_ADDRESS_NODE_ID_H, w0
mov w0, TBLPAG
mov #DESTINATION_ADDRESS_NODE_ID_L, w0
tblwtl w0, [w0]

mov #0x55, w0
mov w0, NVMKEY
mov #0xAA, w0
mov w0, NVMKEY
bset NVMCON, #WR
nop
nop

; fill the latches with data from the block
mov #0x4001, w0
mov w0, NVMCON
mov #DESTINATION_ADDRESS_NODE_ID_H, w0
mov w0, TBLPAG
mov #DESTINATION_ADDRESS_NODE_ID_L, w2

mov node_id_to_write, w0
tblwtl w0, [w2++]
mov battery_charge_voltage_to_write, w0
tblwtl w0, [w2++]

; physically write the data
mov #0x55, w0
mov w0, NVMKEY
mov #0xAA, w0
mov w0, NVMKEY
bset NVMCON, #WR
nop
nop
1:
btsc NVMCON, #WR
bra 1b

return

;===============================================================================
.global _read_ppt_node_id
_read_ppt_node_id:
mov #DESTINATION_ADDRESS_NODE_ID_H, w0
mov w0, TBLPAG
mov #DESTINATION_ADDRESS_NODE_ID_L, w2

tblrdl [w2], w0

return


.global _read_battery_charge_voltage
_read_battery_charge_voltage:
mov #DESTINATION_ADDRESS_BATTERY_CHARGE_VOLTAGE_H, w0
mov w0, TBLPAG
mov #DESTINATION_ADDRESS_BATTERY_CHARGE_VOLTAGE_L, w2

tblrdl [w2], w0

return

.end



