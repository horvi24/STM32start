#ifndef DBG_H
#define DBG_H

//#include <stdint.h>
//#include "stm32g0xx_hal.h"

//#include "stm32g0xx_hal.h"
//#include <stdio.h>
//#include <stdint.h>

//#define DEBUG_MY92XX
#define DEBUG_DMX_DBG1
//#define DEBUG_MY92XX_DBG2
#define DEBUG_CORE_DBG2


void dbg_test();


void dbg_dumppacket(uint8_t *src_packet, uint16_t len);

#endif		/* DBG_H */
