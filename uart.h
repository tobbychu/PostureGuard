/*
 * uart.h
 *
 * Created: 2024/4/11 12:58:25
 *  Author: tobby
 */ 


#ifndef UART_H_
#define UART_H_
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

void UART_init(void);

void UART_send(unsigned char data);

void UART_putstring(char* StringPtr);




#endif /* UART_H_ */