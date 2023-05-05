// compile with 
//      avr-gcc -mmcu=atmega88 fuse_output.c -o fuse_output.elf
// then look at the fuse bits with
//      avr-objdump -s -j .fuse fuse_output.elf
// the first value is the address, followed by [low, high, extended]
//
// you can check what fuses are currently on the board by running a barebones avrdude command
//      avrdude -p atmega88 -P usb -c avrisp2
// write the fuse bits you generated to the board with
//      avrdude -p atmega88 -P usb -c avrisp2 -U efuse:w:0x__:m  -U hfuse:w:0x__:m  -U lfuse:w:0x__:m 
#include <avr/io.h>

FUSES =
{
    .low = LFUSE_DEFAULT 
                | (~FUSE_CKDIV8)  // turn off CKDIV8 fuse, by setting the bit to 1 (Default: 0)
                
                // set full swing crystal oscillator 
                // by setting CKSEL3..1 = 011 (Default: 001),
                //            CKSEL0 = 1 (Default: 0),
                //            SUT1..0 = 11 (Default: 10)
                | (~FUSE_CKSEL2) | (~FUSE_CKSEL0) | (~FUSE_SUT0),
    .high = HFUSE_DEFAULT,
    .extended = EFUSE_DEFAULT,
};

int main() {
    return 0;
}