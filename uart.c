/*
 * uart.c
 *
 * Created: 2024/4/11 12:58:14
 *  Author: tobby
 */ 
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <avr/io.h>

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

void UART_init(void) {
	UBRR0H = (unsigned char)(BAUD_PRESCALER>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
	UCSR0C |= (1<<USBS0);
}

void UART_send(unsigned char data) {
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void UART_putstring(char* StringPtr) {
	while(*StringPtr != 0x00) {
		UART_send(*StringPtr);
		StringPtr++;
	}
}