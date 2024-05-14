#ifndef MY9291_H
#define MY9291_H

#include "main.h"
#include "core.h"

#define DLY_8US   16
#define DLY_12US  26
#define DLY_16US  36

void dly_us(uint16_t time);				//16=8.3us 26=12.2us 36=16.2us

void my92xx_di_pulse(uint16_t times);
void my92xx_dcki_pulse(uint16_t times);
void my92xx_write(uint16_t data, uint8_t bit_length);



#endif		/* MY9291_H */
