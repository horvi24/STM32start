//#include <stdarg.h>
//#include "stdio.h"
//#include <string.h>

//#include "dbg.h"
//#include "usbd_cdc_if.h"
//#include "stm32g0xx_hal.h"
#include "dbg.h"


static char dbg_buf[512];

const char *bit_rep[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};


void print_int(uint16_t byte)
{
	uint8_t tmp;

    printf("\r\n0x%4X", byte);
    tmp = byte >> 8;
    printf(" 0b %s %s", bit_rep[tmp >> 4], bit_rep[tmp & 0x0F]);
    tmp = byte & 0xFF;
    printf(" %s %s ", bit_rep[tmp >> 4], bit_rep[tmp & 0x0F]);

}

void print_byte(uint8_t byte)
{
    printf("%s %s", bit_rep[byte >> 4], bit_rep[byte & 0x0F]);
}

void print_half_byte(uint8_t byte)
{
    printf("%s", bit_rep[byte & 0x0F]);
}

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

