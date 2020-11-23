///
/// 4-Digit Seven Segment Display Driver
///
/// Thom Hayward. Nov 2020.
///

// Define an architecture to appease vscode
#ifndef __AVR_ARCH__
#define __AVR_ATtiny84__
#endif

#ifndef F_CPU
#define F_CPU               8000000UL
#endif

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "ascii.hh"

#define LATCH_PIN PA0
#define CLOCK_PIN PA1
#define DATA_PIN PA2

#define BAUDRATE            9600
#define CYCLES_PER_BIT      (F_CPU / BAUDRATE)
#if (CYCLES_PER_BIT > 255)
#define     DIVISOR         8
#define     PRESCALE        2
#else
#define     DIVISOR         1
#define     PRESCALE        1
#endif
#define FULL_BIT_TICKS      (CYCLES_PER_BIT / DIVISOR)
#define HALF_BIT_TICKS      (FULL_BIT_TICKS / 2)
#define START_DELAY         42
#define TIMER_START_DELAY   (START_DELAY / DIVISOR)

#define DIGITS 8

volatile uint8_t input_idx = 0xFF;
volatile uint8_t output[DIGITS];

static const uint8_t mask[] = {
    0xfe, 0xfd, 0xfb, 0xf7,
    0xef, 0xdf, 0xbf, 0x7f
    };

template<int CLK_PIN, int DAT_PIN> void shift(uint8_t data) {
    for (int bit = 7; bit >= 0; bit--) {
        if (((data >> bit) & 0x01) == 0x01) {
            PORTA |= _BV(DAT_PIN);
        } else {
            PORTA &= ~_BV(DAT_PIN);
        }
        PORTA |= _BV(CLK_PIN);
        PORTA &= ~_BV(CLK_PIN);
    }
}

uint8_t reverse_byte(uint8_t x) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

int main() {

    cli();

    // Setup rx
    DDRA &= ~_BV(PA6);                                          // set PA6/DI as input
    PORTA |= _BV(PA6);                                          // ... with internal pull-up
    USICR = 0;                                                  // disable USI
    GIMSK |=  _BV(PCIE0);                                       // enable pin change interrupts
    PCMSK0 |= _BV(PCINT6);                                      // .. on PA6/DI

    sei();
    
    DDRA |= _BV(LATCH_PIN) | _BV(CLOCK_PIN) | _BV(DATA_PIN);

    

    for (;;) {
        // Ensure even illumination across all digits regardless of how many segments are active by only activating
        // one segment at a time.
        for (uint8_t segment = 0; segment < 8; segment++) {
            for (uint8_t digit = 0; digit < DIGITS; digit++) {
                PORTA &= ~_BV(LATCH_PIN);
                shift<CLOCK_PIN, DATA_PIN>(mask[digit]);
                shift<CLOCK_PIN, DATA_PIN>(output[digit] & _BV(segment));
                PORTA |= _BV(LATCH_PIN);
            }
        }
    }

}

ISR (PCINT0_vect) {                                         
    if ((PINA & _BV(PA6)) == 0) {                               // PA6/DI has gone low; we've probably detected a start bit
        GIMSK &= ~_BV(PCIE0);                                   // disable pin change interrupts
        TCCR0A = _BV(WGM01);                                    // setup timer0 in CTC mode
#if (DIVISOR == 8)
        TCCR0B = _BV(CS01);                                     // set prescaler to cpu_freq/8
#endif
#if (DIVISOR == 1)
        TCCR0B = _BV(CS00);                                     // set prescaler to cpu_freq/1
#endif
        GTCCR |= _BV(PSR10);                                    // reset prescaler
        TCNT0 = 0;                                              // count up from 0
        OCR0A = HALF_BIT_TICKS - TIMER_START_DELAY;             // try to trigger the timer in the middle of a signal
        TIFR0 = _BV(OCF0A);                                     // clear output compare interrupt flag
        TIMSK0 |= _BV(OCIE0A);                                  // enable output compare interrupt
    }
}

ISR (TIM0_COMPA_vect) {
    TIMSK0 &= ~_BV(OCIE0A);                                     // disable COMPA interrupt
    TCNT0 = 0;                                                  // count up from 0
    OCR0A = FULL_BIT_TICKS;                                     // shift every bit width
    USICR = _BV(USIOIE) | _BV(USICS0);
    USISR = _BV(USIOIF) | 0x08;
}

ISR (USI_OVF_vect) {                                            // USI has read eight bits
    uint8_t data = reverse_byte(USIDR);
    if (input_idx == 0xFF) {
        if ((data & 0xF8) == 0xF8) {
            input_idx = data & 0x07;
        }
    } else {
        if (input_idx < DIGITS) {
            output[input_idx] = pgm_read_byte(&ascii_mappings[data]);
        }
        input_idx = 0xFF;
    }    
    USICR = 0;                                                  // disable USI; we've finished reading data
    GIFR = _BV(PCIF0);                                          // clear pin change interrupt flag
    GIMSK |= _BV(PCIE0);                                        // enable pin change interrupts again
}
