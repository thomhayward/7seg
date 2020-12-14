#! /usr/bin/env python3

import serial
import time

ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate = 38400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    timeout=1
)

try:
    while True:
        for x in range(10000):
            ser.write('\x0D{:>4d}'.format(x).encode('utf-8'))
            # time.sleep(0.04)

except KeyboardInterrupt:
    ser.write(b'\x0D')
