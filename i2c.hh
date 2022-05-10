void i2c_init_slave(uint8_t);
uint8_t i2c_bytes_available();
void i2c_write(uint8_t);
uint8_t i2c_read();
void on_receive(void (*function)(uint8_t));
void on_request(void (*function)(void));

#ifndef I2C_RX_BUFFER_SIZE
#define I2C_RX_BUFFER_SIZE (2)
#endif

#define I2C_RX_BUFFER_MASK (I2C_RX_BUFFER_SIZE - 1)

#ifndef I2C_TX_BUFFER_SIZE
#define I2C_TX_BUFFER_SIZE (2)
#endif

#define I2C_TX_BUFFER_MASK (I2C_TX_BUFFER_SIZE - 1)
