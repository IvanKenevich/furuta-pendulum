#include "../../utilities/defines/defs.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDs	PORTD
#define red     0b01100000
#define yellow  0b10100000
#define green   0b11000000

uint8_t adc_data;

ISR(ADC_vect)
{
	adc_data = ADCH;

	if(adc_data > (2*UINT8_MAX)/3)
	{
		LEDs = red;
	}
	else if (adc_data > (1*UINT8_MAX)/3)
	{
		LEDs = yellow;
	}
	else
	{
		LEDs = green;
	}

	PORTC ^= 0b00100000;
}

int main (void)
{
	DDRD   = 0b11100000; // led outputs
    DDRC   = 0b00100000; // speed indicator
	ADMUX  = 0b00100000; // Input on AD Channel 0
	ADCSRA = 0b11101011; // ADC on, auto trigger on, /16 prescaler, interrupt on

	asm("sei"); // enable interrupts

	while(1) {}
}