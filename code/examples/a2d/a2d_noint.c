#include "../../utilities/defines/defs.h"
#include <avr/io.h>
// #define RES10 // disable this to switch to an 8-bit resolution version

// Program runs at 4.38 kHz with 10 bit and 8 bit single channel a2d

#define LEDs    PORTD
#define red     0b01100000
#define yellow  0b10100000
#define green   0b11000000

int main (void)
{
#ifdef RES10
    unsigned int adc_data0;
#else
    uint8_t adc_data0;
#endif

    DDRD   = 0b11100000; // led outputs
    DDRC   = 0b00100000; // speed indicator
#ifdef RES10
    ADMUX  = 0b00000000; //Input on AD Channel 0
#else
    ADMUX  = 0b00100000; //Input on AD Channel 0, left adjust output
#endif 
    ADCSRA = 0b10000111; // ADC on, /128 for a 16 MHz clock, interrupt off

    while(1)
    {
#ifdef RES10
        ADMUX  = 0b00000000; 
#else
        ADMUX  = 0b00100000; 
#endif 
        ADCSRA = ADCSRA | 0b01000000;  // Start AD conversion
        while ((ADCSRA & 0b01000000) == 0b01000000); // Wait while AD conversion is executed
#ifdef RES10
        adc_data0 = ADCW;
#else
        adc_data0 = ADCH;
#endif 

        // ADMUX  = 0b00000001; //Input on AD Channel 1
        // ADCSRA = ADCSRA | 0b01000000;  // Start AD conversion
        // while ((ADCSRA & 0b01000000) == 0b01000000); // Wait while AD conversion is executed
        // adc_data1 = ADCW;

        if (adc_data0 > (2*
        #ifdef RES10 
            UINT16_MAX
        #else  
            UINT8_MAX
        #endif
                )/3)
        {
            LEDs = red;
        }
        else if (adc_data0 > (1*
        #ifdef RES10 
            UINT16_MAX
        #else  
            UINT8_MAX
        #endif
                )/3)
        {
            LEDs = yellow;
        }
        else
        {
            LEDs = green;
        }

        PORTC ^= 0b00100000;
    }
}