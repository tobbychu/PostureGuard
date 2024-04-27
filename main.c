/*
 * main.c
 *
 * Created: 4/11/2024 12:46:01 PM
 *  Author: tobby
 */ 
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#include <stdio.h>
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "posealert.h"

void atmega_init() {
	DDRB &= ~(1 << PORTB2);  // Set button pin as input
	//PORTB &= ~(1 << PORTB2);
	DDRB &= ~(1 << PORTB3);  // Set button pin as input
	//PORTB &= ~(1 << PORTB3);
	DDRC |= (1 << PORTC3); // PC3 as output
	sei();
}

int main(void) {
	Instance m1 = MPU6050_INSTANCE_1, m2 = MPU6050_INSTANCE_2, m3 = MPU6050_INSTANCE_3, m4 = MPU6050_INSTANCE_4;
	i2c_init(0);
	i2c_init(1);
	mpu6050_init(m1);
	mpu6050_init(m2);
	mpu6050_init(m3);
	mpu6050_init(m4);
	UART_init();
	atmega_init();
	
	char buffer[50];
	double roll1, pitch1, yaw1;
	double roll2, pitch2, yaw2;
	double roll3, pitch3, yaw3;
	double roll4, pitch4, yaw4;
	
	while (1) {
		sprintf(buffer, "-------------------\n");
		UART_putstring(buffer);
		
		if (PINB & (1 << PINB2) || PINB & (1 << PINB3)) {
			// Button is pressed, reset angles to zero
			mpu6050_resetQuaternion(m1);
			mpu6050_resetQuaternion(m2);
			mpu6050_resetQuaternion(m3);
			mpu6050_resetQuaternion(m4);
			sprintf(buffer, "Angles reset to zero.\n");
			UART_putstring(buffer);
			_delay_ms(100);  // Debounce delay
		}
		
		
		// Get the roll, pitch, and yaw angles
		mpu6050_getRollPitchYaw(m1, &roll1, &pitch1, &yaw1);

		// Convert angles from radians to degrees
		roll1 = roll1 * 180.0 / M_PI;
		pitch1 = pitch1 * 180.0 / M_PI;
		yaw1 = yaw1 * 180.0 / M_PI;
		
		sprintf(buffer, "Roll1: %.2f, Pitch1: %.2f, Yaw1: %.2f\n", roll1, pitch1, yaw1);
		UART_putstring(buffer);
		
		/***************************/
		// Get the roll, pitch, and yaw angles
		mpu6050_getRollPitchYaw(m2, &roll2, &pitch2, &yaw2);

		// Convert angles from radians to degrees
		roll2 = roll2 * 180.0 / M_PI;
		pitch2 = pitch2 * 180.0 / M_PI;
		yaw2 = yaw2 * 180.0 / M_PI;
		
		sprintf(buffer, "Roll2: %.2f, Pitch2: %.2f, Yaw2: %.2f\n", roll2, pitch2, yaw2);
		UART_putstring(buffer);
		
		/***************************/
		// Get the roll, pitch, and yaw angles
		mpu6050_getRollPitchYaw(m3, &roll3, &pitch3, &yaw3);

		// Convert angles from radians to degrees
		roll3 = roll3 * 180.0 / M_PI;
		pitch3 = pitch3 * 180.0 / M_PI;
		yaw3 = yaw3 * 180.0 / M_PI;
		
		sprintf(buffer, "Roll3: %.2f, Pitch3: %.2f, Yaw3: %.2f\n", roll3, pitch3, yaw3);
		UART_putstring(buffer);
		
		/***************************/
		// Get the roll, pitch, and yaw angles
		mpu6050_getRollPitchYaw(m4, &roll4, &pitch4, &yaw4);

		// Convert angles from radians to degrees
		roll4 = roll4 * 180.0 / M_PI;
		pitch4 = pitch4 * 180.0 / M_PI;
		yaw4 = yaw4 * 180.0 / M_PI;
		
		sprintf(buffer, "Roll4: %.2f, Pitch4: %.2f, Yaw4: %.2f\n", roll4, pitch4, yaw4);
		UART_putstring(buffer);
		
		/*************************/
		
		int poseCode = 0;
		poseCode = checkPosture(roll1, pitch1, yaw1, roll2, pitch2, yaw2,
			roll3, pitch3, yaw3, roll4, pitch4, yaw4);
		printAlert(poseCode);
		
		if (poseCode != 0) {
			PORTC |= (1 << PORTC3);
			_delay_ms(500);
			PORTC &= ~(1 << PORTC3);
		}
		
		sprintf(buffer, "\n");
		UART_putstring(buffer);
		
		_delay_ms(500);
	}
}
