# STM32F103C6T6A I2C Driver

This is an I2C driver for the STM32F103C6T6A microcontroller. The driver provides functions for both master and slave modes, allowing users to easily integrate I2C communication into their STM32 projects.

## Dependencies

Make sure to include the files in the required folder in your project folder:


The driver depends on these files for proper functionality.

## Functions

### Master Mode

#### `void I2C_init_master(int I2C_PINS, unsigned short i2c_mode)`

Initializes the STM32F103C6T6A as a master, allowing users to choose the I2C pins and the communication speed (standard mode 100KHz or fast mode 500KHz).

#### `void I2C_start()`

Generates a start condition for master mode.

#### `void I2C_address(uint8_t address, uint8_t rw)`

Used in master mode to set the address for communication and specify the read or write operation.

#### `void I2C_master_write_byte(uint8_t data)`

Used in master mode to send a single byte of data.

#### `void I2C_master_write_data(uint8_t address, uint8_t data[])`

Used in master mode to send multiple bytes of data.

#### `char I2C_master_rx_byte()`

Used in master mode to request and receive a byte.

#### `void I2C_master_rx_data(uint8_t address, char data[], unsigned short size)`

Used in master mode to receive a limited number of bytes.

#### `void I2C_stop()`

Generates a stop condition for master mode.

### Slave Mode

#### `void I2C_init_slave(int I2C_PINS, unsigned short i2c_mode, uint8_t address)`

Initializes the STM32F103C6T6A as a slave, allowing users to choose the I2C pins, communication speed, and set a 7-bit address for identification on the I2C bus. This function utilizes interrupts, and the user must implement `I2C1_EV_IRQHandler()`.

#### `uint8_t I2C_slave_receive_byte()`

Used in slave mode to receive a single byte. Must be implemented inside `I2C1_EV_IRQHandler()`.

#### `void I2C_slave_receive_data(char Rx_Buffer[], unsigned short size)`

Used in slave mode to receive a specific number of bytes. Must be implemented inside `I2C1_EV_IRQHandler()`.

#### `void I2C_slave_transmit_byte(uint8_t data)`

Used in slave mode to transmit a byte of data. Must be implemented inside `I2C1_EV_IRQHandler()`.

#### `void I2C_slave_transmit_data(char data[])`

Used in slave mode to transmit a set of data. Must be implemented inside `I2C1_EV_IRQHandler()`.

#### `void send_NACK()`

Sends a non-acknowledgment bit.

---

Feel free to reach out for any questions or improvements!
