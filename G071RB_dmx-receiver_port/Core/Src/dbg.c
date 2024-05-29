
#include <dbg.h>
#include "stm32g0xx_hal.h"

static char dbg_buf[512];

void dbg_dumppacket(uint8_t *src_packet, uint16_t len)
{
	char *ptr = dbg_buf;
	uint16_t to_send;

	ptr += sprintf(ptr, "[%lu] Dumping packet (type=%02X len=%d):",
		       HAL_GetTick(), src_packet[0], len);

	for (int i = 1; i < len; i++) {
		if ((i % 16) == 1)
			ptr += sprintf(ptr, "\r\n");

		ptr += sprintf(ptr, "%02X ", src_packet[i]);

		to_send = ptr - dbg_buf;
		if (to_send > 500)
		{
			printf(dbg_buf);
			ptr = dbg_buf;
		}
	}

	ptr += sprintf(ptr, "\r\n");

	to_send = ptr - dbg_buf;
	printf(dbg_buf);

}

