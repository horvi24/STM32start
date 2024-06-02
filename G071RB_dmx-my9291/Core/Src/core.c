//#include "main.h"

#include "core.h"

#define ADC_CHANNELS 	2
#define ADC_MAX 		1800

//#define ADDR_LED_RGBW	1

extern TIM_HandleTypeDef htim3;

static uint8_t packet[576];

bool core_init(void) {
	if (!led_init(&htim3))	//+h24
		return false;

	return true;
}

/* Non-Blocking receive entire DMX packet */
void core_process_h24(void) {
	uint16_t len;

	len = dmx_receive_24(packet);

	if (len == 0) {
		my92xx_update();
	} else {
		//DBG_OUT5();

		led_set(0, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM]));
		led_set(1, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM + 1]));
		led_set(2, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM + 2]));
		led_set(3, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM + 3]));

		my92xx_setChannel(MY92XX_R1, packet[DMX_ADDR_LED_RGBW_MY92]);
		my92xx_setChannel(MY92XX_G1, packet[DMX_ADDR_LED_RGBW_MY92 + 1]);
		my92xx_setChannel(MY92XX_B1, packet[DMX_ADDR_LED_RGBW_MY92 + 2]);
		my92xx_setChannel(MY92XX_W1, packet[DMX_ADDR_LED_RGBW_MY92 + 3]);

		my92xx_setChannel(MY92XX_R2, packet[DMX_ADDR_LED_RGBW_MY92 + 4]);
		my92xx_setChannel(MY92XX_G2, packet[DMX_ADDR_LED_RGBW_MY92 + 5]);
		my92xx_setChannel(MY92XX_B2, packet[DMX_ADDR_LED_RGBW_MY92 + 6]);
		my92xx_setChannel(MY92XX_W2, packet[DMX_ADDR_LED_RGBW_MY92 + 7]);

		my92xx_setChannel(MY92XX_R3, packet[DMX_ADDR_LED_RGBW_MY92 + 8]);
		my92xx_setChannel(MY92XX_G3, packet[DMX_ADDR_LED_RGBW_MY92 + 9]);
		my92xx_setChannel(MY92XX_B3, packet[DMX_ADDR_LED_RGBW_MY92 + 10]);
		my92xx_setChannel(MY92XX_W3, packet[DMX_ADDR_LED_RGBW_MY92 + 11]);

		my92xx_setChannel(MY92XX_R4, packet[DMX_ADDR_LED_RGBW_MY92 + 12]);
		my92xx_setChannel(MY92XX_G4, packet[DMX_ADDR_LED_RGBW_MY92 + 13]);
		my92xx_setChannel(MY92XX_B4, packet[DMX_ADDR_LED_RGBW_MY92 + 14]);
		my92xx_setChannel(MY92XX_W4, packet[DMX_ADDR_LED_RGBW_MY92 + 15]);

		my92xx_update();
	}
}

/* Blocking receive entire DMX packet */
void core_process(void) {

	dmx_receive(packet);

	led_set(0, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM]));
	led_set(1, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM + 1]));
	led_set(2, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM + 2]));
	led_set(3, curve_fn(packet[DMX_ADDR_LED_RGBW_PWM + 3]));

	my92xx_setChannel(MY92XX_R1, packet[DMX_ADDR_LED_RGBW_MY92]);
	my92xx_setChannel(MY92XX_G1, packet[DMX_ADDR_LED_RGBW_MY92 + 1]);
	my92xx_setChannel(MY92XX_B1, packet[DMX_ADDR_LED_RGBW_MY92 + 2]);
	my92xx_setChannel(MY92XX_W1, packet[DMX_ADDR_LED_RGBW_MY92 + 3]);

	my92xx_update();
}
