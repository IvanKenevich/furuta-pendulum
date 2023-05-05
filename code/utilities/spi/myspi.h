#ifndef _MYSPI
#define _MYSPI

#include "../defines/defs.h"
#include <avr/io.h>

void spi_init(void);
unsigned char spi_write_read(unsigned char spi_data);

#endif