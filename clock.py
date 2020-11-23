#! /usr/bin/env python3

import serial
import time

def decode_number(value):
    mapping = {'0': 63, '1': 6, '2': 91, '3': 79, '4': 102, '5': 109, '6': 125, '7': 7, '8': 127, '9': 111, '.': 128, ' ': 0 }
    out = []
    inp = str(value) + ' '
    for index in range(0, len(inp) - 1):
        char = inp[index]
        if char in mapping:
            value = mapping[char]
            if value == 128:
                out[len(out) - 1] |= value
            else:
                out.append(value)
    return out

ser = serial.Serial(
    port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    timeout=1
)

try:
    # Initialise by writing five zeros
    ser.write(b'\x00\x00\x00\x00\x00\x00')
    while True:
        now = decode_number(time.strftime('%H%M'))
        output = [0xAA, 0x03] + now + [0x55]
        ser.write(bytearray(output))
        time.sleep(0.5)
        now = decode_number(time.strftime('%H%M'))
        output = [0xAA, 0x0] + now + [0x55]
        ser.write(bytearray(output))
        time.sleep(0.5)
except KeyboardInterrupt:
    ser.write(b'\xaa\x00\x00\x00\x00\x00\x55')
