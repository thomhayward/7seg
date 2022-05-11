///
/// i2c Slave Driver
///
#include <avr/interrupt.h>
#include <avr/io.h>
#include "i2c.hh"

typedef enum {
    CHECK_ADDRESS,
    SEND_DATA,
    SEND_DATA_ACK_WAIT,
    SEND_DATA_ACK_CHECK,
    RECV_DATA,
    RECV_DATA_ACK,
} i2c_slave_state_t;

i2c_slave_state_t i2c_slave_state = CHECK_ADDRESS;

uint8_t i2c_slave_address = 0;
bool i2c_slave_register_received = false;
uint8_t i2c_slave_register_address = 0;
uint8_t i2c_slave_register_bank_len = 0;
uint8_t **i2c_slave_register_bank = 0;
void (*i2c_slave_callback)(uint8_t) = 0;

#define USI_USICR_START_CONDITION() ((1<<USISIE)|(0<<USIOIE)|(1<<USIWM1)|(0<<USIWM0)|(1<<USICS1)|(0<<USICS0)|(0<<USICLK)|(0 << USITC))
#define USI_USISR_START_CONDITION() ((1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC))

void i2c_slave_init(uint8_t address, uint8_t **register_bank, uint8_t register_bank_len, void (*callback)(uint8_t)) {
    i2c_slave_state = CHECK_ADDRESS;
    i2c_slave_address = address;
    i2c_slave_register_received = false;
    i2c_slave_register_bank = register_bank;
    i2c_slave_register_bank_len = register_bank_len;
    i2c_slave_callback = callback;
    // setup SCL & SDA
    DDR_USI |= (1<<PORT_USI_SDA)|(1<<PORT_USI_SCL);   // set SDA, SCL input
    PORT_USI |= (1<<PORT_USI_SDA)|(1<<PORT_USI_SCL);  // set SDA, SCL pull-up
    DDR_USI &= ~(1<<PORT_USI_SDA);                        // set SDA ouput ????
    // setup USI
    USICR = USI_USICR_START_CONDITION();
	USISR = USI_USISR_START_CONDITION();
}

ISR(USI_START_VECTOR) {
    // start condition detected
    i2c_slave_state = CHECK_ADDRESS;
    // set SDA output
    PORT_USI |= (1<<PORT_USI_SDA);
    DDR_USI &= ~(1<<PORT_USI_SDA);
    // wait for SCL to go low
    while ((PIN_USI & (1<<PIN_USI_SCL)) && !((PIN_USI & (1<<PIN_USI_SDA)))) { }
    // test if the start condition completed
    if (!(PIN_USI & (1<<PIN_USI_SDA))) {
        // enable the overflow interrupt
        USICR = (1<<USISIE)|(1<<USIOIE)|(1<<USIWM1)|(1<<USIWM0)|(1<<USICS1)|(0<<USICS0)|(0<<USICLK)|(0<<USITC);
    } else {
        // we received a stop condition instead
        USICR = USI_USICR_START_CONDITION();
    }
    // clear flags by writing a 1
    USISR = USI_USISR_START_CONDITION() | (0x0<<USICNT0);
}

ISR(USI_OVERFLOW_VECTOR) {
    switch (i2c_slave_state) {
        case CHECK_ADDRESS:
            if ((USIDR == 0) || ((USIDR >> 1) == i2c_slave_address)) {
                if ((USIDR & 0x01) == 0x01) {
                    i2c_slave_state = SEND_DATA;
                } else {
                    i2c_slave_register_received = false;
                    i2c_slave_state = RECV_DATA;
                }
                // configure USI to send ACK for one clock-cycle
                USIDR = 0;
                DDR_USI |= (1<<PORT_USI_SDA);
                USISR = (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|(0x0E<<USICNT0);
            } else {
                // not us, configure USI to wait for start condition
                USICR = USI_USICR_START_CONDITION();
                USISR = USI_USISR_START_CONDITION() | (0x0<<USICNT0);
            }
            break;
        case SEND_DATA:
            i2c_slave_state = SEND_DATA_ACK_WAIT;
            if (i2c_slave_register_address < i2c_slave_register_bank_len) {
                USIDR = *(i2c_slave_register_bank[i2c_slave_register_address]);
                i2c_slave_register_address += 1;
            } else {
                USIDR = 0;
            }
            // configure USI to send contents of USIDR
            DDR_USI |= (1<<PORT_USI_SDA);
            USISR = (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|(0x0<<USICNT0);
            break;
        case SEND_DATA_ACK_WAIT:
            i2c_slave_state = SEND_DATA_ACK_CHECK;
            // configure USI to receive an ACK
            DDR_USI &= ~(1<<PORT_USI_SDA);
            USIDR = 0;
            USISR = (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|(0x0E<<USICNT0);
            break;
        case SEND_DATA_ACK_CHECK:
            if (USIDR) {
                USICR = USI_USICR_START_CONDITION();
                USISR = USI_USISR_START_CONDITION() | (0x0<<USICNT0);
            }
            break;
        case RECV_DATA:
            // configure USI to receive a byte
            i2c_slave_state = RECV_DATA_ACK;
            DDR_USI &= ~(1<<PORT_USI_SDA);
            USISR = (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|(0x0<<USICNT0);
            break;
        case RECV_DATA_ACK:
            i2c_slave_state = RECV_DATA;
            if (!i2c_slave_register_received) {
                i2c_slave_register_address = USIDR;
                i2c_slave_register_received = true;
            } else if (i2c_slave_register_address < i2c_slave_register_bank_len) {
                *(i2c_slave_register_bank[i2c_slave_register_address]) = USIDR;
                // trigger the callback
                if (i2c_slave_callback) {
                    i2c_slave_callback(i2c_slave_register_address);
                }
            }
            // configure USI to send an ACK for one clock cycle
            USIDR = 0;
            DDR_USI |= (1<<PORT_USI_SDA);
            USISR = (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|(0x0E<<USICNT0);
            break;
    }
}
