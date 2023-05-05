#ifndef _MYUART
#define _MYUART

#include "myuart.h"

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void uart_init() {
    UBRR0H = (MYUBRR) >> 8;     // set baud rate
    UBRR0L = MYUBRR;            // set baud rate
    UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable receiver and transmitter
    
    UCSR0C = 0b00101110; // async USART, even parity, 2 stop bits, 8 bit char size, 

    stdout = &mystdout; // Required for printf init
}

// USART Functions
int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0); // loop until data register is empty
    UDR0 = c;                             // load new data to transmit
    
    return 0;
}

unsigned char uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}

#endif // _MYUART