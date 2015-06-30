#include "UART1.h"

void init_uart1(uint32_t baudrate)
{
    uint16_t UBRR_val = ((F_CPU/16)/(baudrate))-1;
    UBRR1H = UBRR_val >> 8;
    UBRR1L = UBRR_val;
    UCSR1B |= (1<<TXEN1) | (1<<RXEN1); // UART TX (Transmit - senden) einschalten
    UCSR1C |= (1<<USBS1) | (3<<UCSZ10); //Modus Asynchron 8N1 (8 Datenbits, No Parity, 1 Stopbit)
}

void uart1_putc(unsigned char c)
{
    while(!(UCSR1A & (1<<UDRE1))); // wait until sending is possible
    UDR1 = c; // output character saved in c
}

void uart1_puts(char *s)
{
    while(*s)
    {
        uart1_putc(*s);
        s++;
    }
}
