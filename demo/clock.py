import smbus;
import sys;
import time

i2c_address = 0x20
i2c_channel = 1

def show(bus, address, text):
    for index, char in enumerate(text):
        bus.write_byte_data(address, index, ord(char))
        val = bus.read_byte_data(address, index)
        if val != ord(char):
            print(f"err {val} ({chr(val)}) != {ord(char)} ({char})")

def run_clock(bus, address):
    try:
        tick = False
        while True:
            now = time.strftime('%H%M')
            tick = not tick
            if tick:
                now += ':'
            else:
                now += ' '
            show(bus, address, now)
            time.sleep(1)

    except KeyboardInterrupt:
        show(bus, address, '     ')

run_clock(smbus.SMBus(i2c_channel), i2c_address)
