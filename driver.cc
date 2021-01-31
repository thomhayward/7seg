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

#define SN595_SRCLK_PIN PB1
#define SN595_RCLK_PIN PB3
#define SN595_SER_PIN PB4

#define USI_SDA PB0
#define USI_SCL PB2

#define DIGITS 5

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
    USISR = _BV(USISIF) | _BV(USIOIF) | _BV(USIPF) | _BV(USIDC);
    USICR = _BV(USISIE) | _BV(USIWM1) | _BV(USICS1);

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

#define COPY_BUFFER() {\
    buffer_idx = 0;\
    for (uint8_t i = 0; i < DIGITS; i++) {\
        output[i] = pgm_read_byte(&ascii_mappings[buffer[i]]);\
    }\
}

ISR(USI_START_vect) {
    if ((USISR & _BV(USIPF)) == _BV(USIPF) || buffer_idx == DIGITS) {
        // Stop condition has occurred. Copy buffer to output.
        COPY_BUFFER();
    }
    buffer_idx = 0;
    USISR = _BV(USISIF) | _BV(USIPF) | (0 << USICNT0);
    USICR = _BV(USISIE) | _BV(USIOIE) | _BV(USIWM1) | _BV(USICS1);
}

ISR(USI_OVF_vect) {
    if (buffer_idx < DIGITS) {
        buffer[buffer_idx] = USIBR;
        buffer_idx += 1;
    }
    if ((USISR & _BV(USIPF)) == _BV(USIPF) || buffer_idx == DIGITS) {
        // Stop condition has occurred. Copy buffer to output.
        COPY_BUFFER();
        // Reset the USI.
        USISR = _BV(USISIF) | _BV(USIOIF) | _BV(USIPF) | _BV(USIDC) | (0 << USICNT0);
        USICR = _BV(USISIE) | _BV(USIWM1) | _BV(USICS1);
        return;
    }
    USISR = _BV(USIOIF) | (0 << USICNT0);
}
