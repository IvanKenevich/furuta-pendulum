#include "myspi.h"

void spi_init(void) 
{
    sbi(DDRB, 2); // set SS pin as output
    sbi(DDRB, 3); // set MOSI pin as output
    cbi(DDRB, 4); // set MISO pin as input
    sbi(DDRB, 5); // set SCK pin as output

    SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | (1 << SPR1) | (0 << SPR0);
    SPSR = (0 << SPI2X);
}

unsigned char spi_write_read(unsigned char spi_data)
{
    SPDR=spi_data;
    while ((SPSR & (1<<SPIF))==0);	// Wait until the data transfer is complete
    return SPDR;
}