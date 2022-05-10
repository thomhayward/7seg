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

#define I2C_ADDRESS (0x20)
#define USI_SDA PB0
#define USI_SCL PB2

#define DIGITS 5

volatile bool has_register = false;
volatile uint8_t buffer_idx = 0;
volatile uint8_t buffer[DIGITS];
volatile uint8_t output[DIGITS];

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

void request_isr();
void receive_isr(uint8_t count);

int main() {

    cli();

    // Set the system clock prescaler to 1.
    CLKPR = _BV(CLKPCE);
    CLKPR = 0;

    // Ensure display is blank on startup.
    for (uint8_t i = 0; i < DIGITS; i++) {
        output[i] = 0;
    }

    DDRB = _BV(SN595_SRCLK_PIN) | _BV(SN595_RCLK_PIN) | _BV(SN595_SER_PIN);

    // Initialise the universal serial interface.
    // USISR = _BV(USISIF) | _BV(USIOIF) | _BV(USIPF) | _BV(USIDC);
    // USICR = _BV(USISIE) | _BV(USIWM1) | _BV(USICS1);
    i2c_init_slave(I2C_ADDRESS);
    on_request(request_isr);
    on_receive(receive_isr);

    sei();

    output[0] = pgm_read_byte(&ascii_mappings['1']);
    output[1] = pgm_read_byte(&ascii_mappings['2']);
    output[2] = pgm_read_byte(&ascii_mappings['3']);
    output[3] = pgm_read_byte(&ascii_mappings['4']);

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

void process() {
    uint8_t byte = i2c_read();
    if (has_register) {
        if (buffer_idx < DIGITS) {
            output[buffer_idx] = pgm_read_byte(&ascii_mappings[byte]);
            i2c_write(byte);
        }
    } else {
        buffer_idx = byte;
        has_register = true;
    }
}

void request_isr() {
    has_register = false;
}

void receive_isr(uint8_t count) {
    switch (count) {
        case 0:
            output[0] = pgm_read_byte(&ascii_mappings['E']);
            break;
        case 1:
            process();
            break;
        case 2:
            process();
            process();
            break;
        default:
            output[0] = pgm_read_byte(&ascii_mappings['F']);
    }
    
}
