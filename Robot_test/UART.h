#ifndef UART_H_
#define UART_H_
//#define F_CPU 16000000UL
#include <avr/io.h>
void init_uart(uint32_t baudrate);
void uart_putc(unsigned char c);
void uart_puts(char *s);
#endif /* UART_H_ */