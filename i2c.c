/*
 * i2c.c
 *
 * Created: 2024/4/14 18:25:01
 *  Author: tobby
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#include <stdio.h>
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>


void i2c_init(uint8_t interface) {
	if (interface == 0) {
		TWSR0 = 0x00; // Set prescaler bits to zero
		TWBR0 = 0x0C; // SCL frequency is 400kHz
		TWCR0 = (1 << TWEN); // Enable TWI
	} else if (interface == 1) {
		TWSR1 = 0x00;
		TWBR1 = 0x0C;
		TWCR1 = (1 << TWEN);
	}
}

void i2c_start(uint8_t interface) {
	if (interface == 0) {
		TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
		while (!(TWCR0 & (1 << TWINT))); // Wait till complete TWINT is set
	} else if (interface == 1) {
		TWCR1 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
		while (!(TWCR1 & (1 << TWINT))); // Wait till complete TWINT is set
	}
}

void i2c_stop(uint8_t interface) {
	if (interface == 0) {
		TWCR0 = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	} else if (interface == 1) {
		TWCR1 = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	}
}

void i2c_write(uint8_t interface, uint8_t u8data) {
	if (interface == 0) {
		TWDR0 = u8data;
		TWCR0 = (1 << TWINT) | (1 << TWEN);
		while (!(TWCR0 & (1 << TWINT))); // Wait till complete TWINT is set
	} else if (interface == 1) {
		TWDR1 = u8data;
		TWCR1 = (1 << TWINT) | (1 << TWEN);
		while (!(TWCR1 & (1 << TWINT))); // Wait till complete TWINT is set
	}
}

uint8_t i2c_readAck(uint8_t interface) {
	if (interface == 0) {
		TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
		while (!(TWCR0 & (1 << TWINT))); // Wait till complete TWINT is set
		return TWDR0;
	} else if (interface == 1) {
		TWCR1 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
		while (!(TWCR1 & (1 << TWINT))); // Wait till complete TWINT is set
		return TWDR1;
	}
}

uint8_t i2c_readNak(uint8_t interface) {
	if (interface == 0) {
		TWCR0 = (1 << TWINT) | (1 << TWEN);
		while (!(TWCR0 & (1 << TWINT))); // Wait till complete TWINT is set
		return TWDR0;
	} else if (interface == 1) {
		TWCR1 = (1 << TWINT) | (1 << TWEN);
		while (!(TWCR1 & (1 << TWINT))); // Wait till complete TWINT is set
		return TWDR1;
	}
}