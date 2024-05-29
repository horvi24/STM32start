#include <stdint.h>
#include "main.h"

#include "core.h"
#include "curve.h"
#include "dmx_receiver.h"
#include "led.h"
#include "dbg.h"

#define ADC_CHANNELS 	2
#define ADC_MAX 		1800

#define ADDR_LED_RGBW	1

//-h24 extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

static uint8_t packet[576];

bool core_init(void)
{
	if (!led_init(&htim3))	//+h24
		return false;

	return true;
}

void core_process(void)
{
	uint16_t len;

	len = dmx_receive(packet);

	led_set(0, curve_fn(packet[ADDR_LED_RGBW]));
	led_set(1, curve_fn(packet[ADDR_LED_RGBW+1]));
	led_set(2, curve_fn(packet[ADDR_LED_RGBW+2]));
	led_set(3, curve_fn(packet[ADDR_LED_RGBW+3]));

	if(!HAL_GPIO_ReadPin(SW_BLUE_GPIO_Port, SW_BLUE_Pin)){
		  dbg_dumppacket(packet, len);
	}

}
