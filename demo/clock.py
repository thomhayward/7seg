#! /usr/bin/env python3

import serial
import time

ser = serial.Serial(
    port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
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
        ser.write(time.strftime('\x0D%H%M:').encode('ascii'))
        time.sleep(1)
        ser.write(time.strftime('\x0D%H%M').encode('ascii'))
        time.sleep(1)

except KeyboardInterrupt:
    ser.write(b'\x0D    ')