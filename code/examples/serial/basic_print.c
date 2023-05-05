#include "../../utilities/defines/defs.h"
#include "../../utilities/uart/myuart.h"
#include <avr/io.h>
#include <avr/delay.h>

#define LEDs    PORTD
#define red     0b01100000
#define yellow  0b10100000
#define green   0b11000000

int main (void)
{
    DDRC   = 0b00100000; // speed indicator

    uart_init();

    while(1)
    {
        printf("a\n");
        PORTC ^= 0b00100000;
    }
}