///
/// 4-Digit Seven Segment Display Driver
///
/// Thom Hayward. Nov 2020.
///

// Define an architecture to appease vscode
#ifndef __AVR_ARCH__
#define __AVR_ATtiny85__
#endif

#ifndef F_CPU
#define F_CPU               8000000UL
#endif

#include <avr/interrupt.h>
#include <avr/io.h>
#include "ascii.hh"
#include "i2c.hh"

#define SN595_SRCLK_PIN PB1
#define SN595_RCLK_PIN PB3
#define SN595_SER_PIN PB4

#define I2C_ADDRESS 0x20

#define DIGITS 5

uint8_t buffer[DIGITS];
uint8_t output[DIGITS];

static const uint8_t mask[] = { 0xFE, 0xFD, 0xF7, 0xEF, 0xFB };

template<int CLK_PIN, int DAT_PIN> void shift(uint8_t data) {
    for (int bit = 7; bit >= 0; bit--) {
        PORTB &= ~_BV(CLK_PIN);
        if (((data >> bit) & 0x01) == 0x01) {
            PORTB |= _BV(DAT_PIN);
        } else {
            PORTB &= ~_BV(DAT_PIN);
        }
        PORTB |= _BV(CLK_PIN);
    }
}

void i2c_callback(uint8_t address);

int main() {

    cli();

    // Set the system clock prescaler to 1.
    CLKPR = _BV(CLKPCE);
    CLKPR = 0;

    // Ensure display is blank on startup.
    for (uint8_t i = 0; i < DIGITS; i++) {
        buffer[i] = 0;
        output[i] = 0;
    }

    // configure pins for SN595 register
    DDRB = _BV(SN595_SRCLK_PIN) | _BV(SN595_RCLK_PIN) | _BV(SN595_SER_PIN);

    // setup registers array for i2c
    uint8_t* registers[DIGITS];
    for (uint8_t i = 0; i < DIGITS; i++) { registers[i] = &buffer[i]; }
    // setup i2c
    i2c_slave_init(I2C_ADDRESS, registers, DIGITS, i2c_callback);

    sei();

    for (;;) {
        // Ensure even illumination across all digits regardless of how many segments are active by only activating
        // one segment at a time.
        for (uint8_t segment = 0; segment < 8; segment++) {
            for (uint8_t digit = 0; digit < DIGITS; digit++) {
                PORTB &= ~_BV(SN595_SRCLK_PIN);
                shift<SN595_RCLK_PIN, SN595_SER_PIN>(mask[digit]);
                shift<SN595_RCLK_PIN, SN595_SER_PIN>(output[digit] & _BV(segment));
                PORTB |= _BV(SN595_SRCLK_PIN);
            }
        }
    }

}

inline void i2c_callback(uint8_t reg_addr) {
    output[reg_addr] = pgm_read_byte(&ascii_mappings[buffer[reg_addr]]);
}
