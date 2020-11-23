#! /usr/bin/env python3

import serial
import time
import threading

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

def output(codes):
    for x in range(1, 5):
        ser.write(bytearray([0xF8 | 5 - x, codes[x]]))

end = False
sepOn = False
def blink_seperator():
    global sepOn, ser, end
    if sepOn:
        sepOn = False
    else:
        sepOn = True
    if sepOn:
        ser.write(bytearray([0xF8 | 5, ord(':')]))
    else:
        ser.write(bytearray([0xF8 | 5, 0]))
    if not end:
        threading.Timer(1, blink_seperator).start()
    
blink_seperator()

try:
    while True:
        now = time.strftime('%H%M')
        for x in range(4):
            ser.write(bytearray([0xF8 | 4 - x, ord(now[x])]))
        time.sleep(5)
except KeyboardInterrupt:
    end = True
    time.sleep(1)
    for x in range(0, 5):
        ser.write(bytearray([0xF8 | x, 0x00]))
