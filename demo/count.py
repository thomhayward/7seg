import smbus;
import sys;

i2c_address = 0x20
i2c_channel = 1

def show(bus, address, text):
    for index, char in enumerate(text):
        bus.write_byte_data(address, index, ord(char))
        val = bus.read_byte_data(address, index)
        if val != ord(char):
            print(f"err {val} ({chr(val)}) != {ord(char)} ({char})", file=sys.stderr)

def run_count(bus, address):
    try:
        tick = False
        while True:
            for x in range(10000):
                show(bus, address, '{:>4d}'.format(x))

    except KeyboardInterrupt:
        show(bus, address, '     ')

run_count(smbus.SMBus(i2c_channel), i2c_address)
