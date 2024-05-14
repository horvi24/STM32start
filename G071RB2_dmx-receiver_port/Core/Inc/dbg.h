#ifndef DBG_H
#define DBG_H

//#include <stdint.h>
//#include "stm32g0xx_hal.h"

//#include "stm32g0xx_hal.h"
#include <stdio.h>
#include <stdint.h>



void dbg_test();


void dbg_dumppacket(uint8_t *src_packet, uint16_t len);

#endif		/* DBG_H */
