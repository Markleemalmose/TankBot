//
//  UART1.h
//  Robot_test
//
//  Created by Mark Lee Malmose on 18/05/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//

#ifndef UART1_H_
#define UART1_H_
//#define F_CPU 16000000UL
#include <avr/io.h>
void init_uart1(uint32_t baudrate);
void uart1_putc(unsigned char c);
void uart1_puts(char *s);
#endif /* UART1_H_ */

