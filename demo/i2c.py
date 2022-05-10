import smbus;
import time

def show(bus, address, text):
    for index, char in enumerate(text):
        bus.write_byte_data(address, index, ord(char))
        val = bus.read_byte_data(address, index)
        if val != ord(char):
            print("Error!")

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

run_clock(smbus.SMBus(1), 0x20)
