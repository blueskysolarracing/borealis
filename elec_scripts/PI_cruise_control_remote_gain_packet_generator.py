"""
This script outputs the complete data packet to send over serial in order to remotely adjust PI gains when developing the cruise control algorithm.
Multi-byte data (16b, 32b) is big-endian (MSB at lowest memory address)
"""
import struct
import serial

#--- SERIAL ---#
baudrate = 115200
port = "/dev/ttys014"

#--- PACKET ---#
PAYLOAD_LENGTH = 1 + 2 + 1 + 3*4 #Data ID, 16b password, 1B blank, 32b k_p, 32b k_i, 32b k_d
START_BYTE = 0xA5
CHASE_ID = 0x5
SEQUENCE_NUMBER = 0
CHASE_CRUISE_PI_GAIN_ID = 5
PASSWORD = 533
PASSWORD_BYTES = PASSWORD.to_bytes(length=2, byteorder="big")
GAIN_SCALE_FACTOR = 100000
packet = [  START_BYTE, PAYLOAD_LENGTH, CHASE_ID, SEQUENCE_NUMBER,
            CHASE_CRUISE_PI_GAIN_ID, PASSWORD_BYTES[0], PASSWORD_BYTES[1], 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,  ]

k_p = 40
k_i = 0.01
k_d = 0.0

#k_p
gain_input = input(f"Enter k_p (default: {k_p}) ")
if gain_input:   k_p = float(gain_input)
k_p = int(GAIN_SCALE_FACTOR*k_p)
byte_array = list(struct.pack('I', k_p))
for i in range(4):  packet[8+i] = byte_array[3-i]

#k_i
gain_input = input(f"Enter k_i (default: {k_i}) ")
if gain_input:   k_i = float(gain_input)
k_i = int(GAIN_SCALE_FACTOR*k_i)
byte_array = list(struct.pack('I', k_i))
for i in range(4):  packet[12+i] = byte_array[3-i]

#k_d
gain_input = input(f"Enter k_d (default: {k_d}) ")
if gain_input:   k_d = float(gain_input)
k_d = int(GAIN_SCALE_FACTOR*k_d)
byte_array = list(struct.pack('I', k_d))
for i in range(4):  packet[16+i] = byte_array[3-i]

print(f"Packet: {packet}")

#send over serial
serialPort = serial.Serial(baudrate=baudrate, port=port)
print(bytearray(packet))
serialPort.write(bytearray(packet))
