#! /usr/bin/env python3
import RPi.GPIO as GPIO
import time

def convert(value, bit):
    if value & (1 << bit) == 0:
        return GPIO.LOW
    return GPIO.HIGH

class SMBus(object):

    pin_SCL = 0
    pin_SDA = 0
    delay = 000000

    def __init__(self, bus=-1):
        GPIO.setwarnings(True)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(12, GPIO.OUT)
        GPIO.output(12, GPIO.HIGH)
        time.sleep(0.33)
        self.set_pin(23, 24)

    def set_pin(self, SDA, SCL):
        self.pin_SCL = SCL
        self.pin_SDA = SDA
        time.sleep(self.delay)

    def start(self):
        GPIO.setup([self.pin_SCL, self.pin_SDA], GPIO.OUT)
        GPIO.output(self.pin_SDA, GPIO.HIGH)
        GPIO.output(self.pin_SCL, GPIO.HIGH)
        time.sleep(self.delay)
        GPIO.output(self.pin_SDA, GPIO.LOW)
        time.sleep(self.delay)

    def stop(self):
        GPIO.setup([self.pin_SCL, self.pin_SDA], GPIO.OUT)
        GPIO.output(self.pin_SCL, GPIO.HIGH)
        time.sleep(self.delay / 2)
        GPIO.output(self.pin_SDA, GPIO.HIGH)
        time.sleep(self.delay * 2)
        GPIO.setup([self.pin_SCL, self.pin_SDA], GPIO.IN)

    def send_byte(self, byte):
        GPIO.setup([self.pin_SCL, self.pin_SDA], GPIO.OUT)
        for i in range(0, 8):
            GPIO.output(self.pin_SCL, GPIO.LOW)
            GPIO.output(self.pin_SDA, convert(byte, 7 - i))
            GPIO.output(self.pin_SCL, GPIO.HIGH)
            time.sleep(self.delay)

smb = SMBus()
try:
    tick = False
    while True:
        smb.start()
        now = time.strftime('%H%M')
        tick = not tick
        if tick:
            now += ':'
        else:
            now += ' '
        for byte in [ord(x) for x in now ]:
            smb.send_byte(byte)
        smb.stop()
        time.sleep(1)

except KeyboardInterrupt:
    smb.start()
    for byte in [ord(x) for x in '     ' ]:
        smb.send_byte(byte)
    smb.stop()
    GPIO.cleanup()