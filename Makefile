
PROJECT = driver

MCU ?= attiny84
PORT ?= $(firstword $(wildcard /dev/cu.usbmodem*))

CXX = avr-g++
CXXSTANDARD = -std=gnu++11
CXXEXTRA = -O3 -Wall -fno-exceptions -ffunction-sections -fdata-sections
CXXFLAGS = $(CXXSTANDARD) $(CXXEXTRA) -mmcu=$(MCU)

OBJCOPY = avr-objcopy

%.elf: $(wildcard *.cc)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	avr-size -C --mcu=$(MCU) $<

.PHONY: build
build: $(PROJECT).hex
	#

.PHONY: upload
upload: $(PROJECT).hex
	# avrdude -p $(MCU) -c arduino -P $(PORT) -D -U $<
	avrdude -v -p $(MCU) -c linuxgpio -U $<

.PHONY: clean
clean:
	rm $(PROJECT).hex
