#include "../defines/defs.h"
#include <stdio.h>
#include <avr/io.h>

#define FOSC F_CPU  // CPU Clock Frequency must be set correctly for the USART to work
#define MYUBRR FOSC/16/BAUD-1

void    uart_init(void);
int     uart_putchar(char c, FILE *stream);
unsigned char uart_getchar(void);