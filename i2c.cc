
#include <avr/interrupt.h>
#include <avr/io.h>
#include "i2c.hh"

#if defined(__AVR_ATtiny85__)
#define DDR_USI             DDRB
#define PORT_USI            PORTB
#define PIN_USI             PINB
#define PORT_USI_SDA        PB0
#define PORT_USI_SCL        PB2
#define PIN_USI_SDA         PINB0
#define PIN_USI_SCL         PINB2
#define USI_START_COND_INT  USISIF
#define USI_START_VECTOR    USI_START_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#define SET_USI_TO_SEND_ACK() {                                 \
  USIDR    =  0;                                                \
  DDR_USI |= (1 << PORT_USI_SDA);                               \
  USISR    = (0 << USI_START_COND_INT)  | (1 << USIOIF) |       \
             (1 << USIPF) | (1 << USIDC)| (0x0E << USICNT0);    \
}

#define SET_USI_TO_READ_ACK() {                                     \
    DDR_USI &= ~(1 << PORT_USI_SDA);                                \
    USIDR    =   0;                                                 \
    USISR    =  (0 << USI_START_COND_INT)   | (1 << USIOIF) |       \
                (1 << USIPF) | (1 << USIDC) | (0x0E << USICNT0);    \
}

#define SET_USI_TO_TWI_START_CONDITION_MODE() {                                   \
    USICR = (1 << USISIE) | (0 << USIOIE) | (1 << USIWM1) | (0 << USIWM0) |       \
            (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);         \
    USISR = (0 << USI_START_COND_INT) | (1 << USIOIF) |                           \
            (1 << USIPF)  | (1 << USIDC)  | (0x0 << USICNT0);                     \
}

#define SET_USI_TO_SEND_DATA() {                                              \
    DDR_USI |= (1 << PORT_USI_SDA);                                           \
    USISR    = (0 << USI_START_COND_INT)    | (1 << USIOIF) |                 \
               (1 << USIPF) | ( 1 << USIDC) | ( 0x0 << USICNT0 );             \
}

#define SET_USI_TO_READ_DATA() { \
    DDR_USI &= ~(1 << PORT_USI_SDA);                                          \
    USISR    =  (0 << USI_START_COND_INT)   | (1 << USIOIF) |                 \
                (1 << USIPF) | (1 << USIDC) | (0x0 << USICNT0);               \
}

#define USI_RECEIVE_CALLBACK() {                                              \
    if (usi_on_receive_callback) {                                                  \
        if (rx_count) {                              \
            usi_on_receive_callback(rx_count);             \
        }                                                                     \
    }                                                                         \
}

#define ONSTOP_USI_RECEIVE_CALLBACK() {                                       \
    if (USISR & (1 << USIPF)) {                                               \
        USI_RECEIVE_CALLBACK();                                               \
    }                                                                         \
}

#define USI_REQUEST_CALLBACK() {                                              \
    USI_RECEIVE_CALLBACK();                                                   \
    if (use_on_receive_callback) { \
        use_on_receive_callback();                                  \
    }\
}

typedef enum {
    USI_SLAVE_CHECK_ADDRESS                = 0x00,
    USI_SLAVE_SEND_DATA                    = 0x01,
    USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA = 0x02,
    USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   = 0x03,
    USI_SLAVE_REQUEST_DATA                 = 0x04,
    USI_SLAVE_GET_DATA_AND_SEND_ACK        = 0x05
} overflowState_t;

static uint8_t slaveAddress;
static volatile overflowState_t overflowState;

void (*use_on_receive_callback)(void);
void (*usi_on_receive_callback)(uint8_t);

static uint8_t rx_buffer[I2C_RX_BUFFER_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint8_t rx_count;

static uint8_t tx_buffer[I2C_TX_BUFFER_SIZE];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_count;

void i2c_init_slave(uint8_t address) {
    rx_head = 0;
    rx_tail = 0;
    rx_count = 0;
    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;
    slaveAddress = address;
    //
    DDR_USI  |=  (1 << PORT_USI_SCL) | (1 << PORT_USI_SDA);
    PORT_USI |=  (1 << PORT_USI_SCL);
    PORT_USI |=  (1 << PORT_USI_SDA);
    DDR_USI  &= ~(1 << PORT_USI_SDA);
    USICR     =  (1 << USISIE) | (0 << USIOIE) | (1 << USIWM1) | (0 << USIWM0) |
                    (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);
    USISR     =  (1 << USI_START_COND_INT) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);
}

void i2c_write(uint8_t data) {
    while (tx_count == I2C_TX_BUFFER_SIZE);
    tx_buffer[tx_head] = data;
    tx_head = (tx_head + 1) & I2C_TX_BUFFER_MASK;
    tx_count++;
}

uint8_t i2c_read() {
    uint8_t rtn_byte;
    while (!rx_count);
    rtn_byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) & I2C_RX_BUFFER_MASK;
    rx_count--;
    return rtn_byte;
}

uint8_t i2c_bytes_available() {
    return rx_count;
}

void on_receive(void (*function)(uint8_t)) {
    usi_on_receive_callback = function;
}

void on_request(void (*function)(void)) {
    use_on_receive_callback = function;
}

ISR(USI_START_VECTOR) {
    overflowState = USI_SLAVE_CHECK_ADDRESS;
    DDR_USI &= ~(1 << PORT_USI_SDA);
    while ((PIN_USI & (1 << PIN_USI_SCL)) && !((PIN_USI & (1 << PIN_USI_SDA))));
    if (!(PIN_USI & (1 << PIN_USI_SDA))) {
        USICR = (1 << USISIE) | (1 << USIOIE) | (1 << USIWM1) | (1 << USIWM0) |
                (1 << USICS1) | (0 << USICS0) | (0 << USICLK) | (0 << USITC);
    } else {
        USICR = (1 << USISIE) | (0 << USIOIE) | (1 << USIWM1 ) | (0 << USIWM0) |
                (1 << USICS1) | (0 << USICS0) | (0 << USICLK ) | (0 << USITC);
    }
    USISR = (1 << USI_START_COND_INT)   | (1 << USIOIF) |
            (1 << USIPF) | (1 << USIDC) | (0x0 << USICNT0);
}

ISR(USI_OVERFLOW_VECTOR) {
    switch (overflowState) {
        case (USI_SLAVE_CHECK_ADDRESS):
            if ((USIDR == 0) || ((USIDR >> 1) == slaveAddress)) {
                use_on_receive_callback();
                if (USIDR & 0x01) {
                    USI_REQUEST_CALLBACK();
                    overflowState = USI_SLAVE_SEND_DATA;
                } else {
                    overflowState = USI_SLAVE_REQUEST_DATA;
                }
                SET_USI_TO_SEND_ACK();
            } else {
                SET_USI_TO_TWI_START_CONDITION_MODE();
            }
            break;
        case (USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA):
            if (USIDR) {
                SET_USI_TO_TWI_START_CONDITION_MODE();
                return;
            }
        case (USI_SLAVE_SEND_DATA):
            if (tx_count) {
                USIDR = tx_buffer[tx_tail];
                tx_tail = (tx_tail + 1) & I2C_TX_BUFFER_MASK;
                tx_count--;
            } else {
                SET_USI_TO_READ_ACK();
                SET_USI_TO_TWI_START_CONDITION_MODE();
                return;
            }
            overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
            SET_USI_TO_SEND_DATA();
            break;
        case (USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA):
            overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
            SET_USI_TO_READ_ACK();
            break;
        case (USI_SLAVE_REQUEST_DATA):
            overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
            SET_USI_TO_READ_DATA();
            break;
        case (USI_SLAVE_GET_DATA_AND_SEND_ACK):
            if (rx_count < I2C_RX_BUFFER_SIZE) {
                    rx_buffer[rx_head] = USIDR;
                    rx_head = (rx_head + 1) & I2C_RX_BUFFER_MASK;
                    rx_count++;
            }
            overflowState = USI_SLAVE_REQUEST_DATA;
            SET_USI_TO_SEND_ACK();
            break;
    }

}
