//#include <stdarg.h>
//#include "stdio.h"
//#include <string.h>

//#include "dbg.h"
//#include "usbd_cdc_if.h"
//#include "stm32g0xx_hal.h"
#include "core.h"


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

void RGBW_red(void) {
    uint8_t i = 0, up = 1;
    while (HAL_GPIO_ReadPin(SW_BLUE_GPIO_Port, SW_BLUE_Pin)) {

        //DBG_OUT1_H();  DBG_OUT1_L();    //**//**//
        led_set(MY92XX_R1, i);
        my92xx_setChannel(MY92XX_R1, i);
        my92xx_setChannel(MY92XX_R2, i);
        my92xx_setChannel(MY92XX_R3, i);
        my92xx_setChannel(MY92XX_R4, i);
        my92xx_update();
        HAL_Delay(1);

        if (up) {
            i++;
            if (i > 254)
                up = 0;
        } else {
            i--;
            if (i < 1)
                up = 1;
        }
    }
}
