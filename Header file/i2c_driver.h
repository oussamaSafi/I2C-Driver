/*
 * i2c_driver.h
 *
 *  Created on: Nov 18, 2023
 *      Author: PC
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_



#endif /* I2C_DRIVER_H_ */
#include "stm32f1xx.h"
#define STANDARD_MODE 180
#define FAST_MODE 36

#define ACK 1
#define NACK 0

#define DEFAULT_PINS 1
#define REMAP_PINS 0

void I2C_init_master(int I2C_PINS,unsigned short i2c_mode);
void I2C_start();
void I2C_address(uint8_t address, uint8_t rw);
void I2C_master_write_byte(uint8_t data);
void I2C_stop();
void I2C_master_write_data(uint8_t address, uint8_t data[]);
char I2C_master_rx_byte();
void I2C_master_rx_data(uint8_t address,char data[], unsigned short size);
void send_NACK();
void I2C_init_slave(int I2C_PINS,unsigned short i2c_mode, uint8_t address);
uint8_t I2C_slave_receive_byte();
void I2C_slave_receive_data(char Rx_Buffer[],unsigned short size);
void I2C_slave_transmit_byte(uint8_t data);
void I2C_slave_transmit_data(char data[]);
