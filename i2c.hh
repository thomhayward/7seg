#ifndef __I2C_H__
#define __I2C_H__

#if defined(__AVR_ATtiny85__)
#define DDR_USI             DDRB
#define PORT_USI            PORTB
#define PIN_USI             PINB
#define PORT_USI_SDA        PB0
#define PORT_USI_SCL        PB2
#define PIN_USI_SDA         PINB0
#define PIN_USI_SCL         PINB2
#define USI_START_VECTOR    USI_START_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

void i2c_slave_init(
    uint8_t address,
    uint8_t **register_bank,
    uint8_t register_bank_len,
    void (*callback)(uint8_t)
    );

#endif
