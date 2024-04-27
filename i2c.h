/*
 * i2c.h
 *
 * Created: 2024/4/14 18:26:46
 *  Author: tobby
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#ifndef I2C_H_
#define I2C_H_

void i2c_init(uint8_t interface);

void i2c_start(uint8_t interface);

void i2c_stop(uint8_t interface);

void i2c_write(uint8_t interface, uint8_t u8data);

uint8_t i2c_readAck(uint8_t interface);

uint8_t i2c_readNak(uint8_t interface);



#endif /* I2C_H_ */