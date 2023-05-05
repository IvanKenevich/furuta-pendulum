#include "../../utilities/defines/defs.h"
#include <avr/io.h>
#include <util/delay.h>

#define sbi(var, mask)  ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)  ((var) &= (uint8_t)~(1 << mask))

int main(void) {
    DDRD = 0b01000000; // set pin 6 as output

    while(1) {
        sbi(PORTD, 6);
        _delay_ms(500);

        cbi(PORTD, 6);
        _delay_ms(500);
    }

    return 0;
}