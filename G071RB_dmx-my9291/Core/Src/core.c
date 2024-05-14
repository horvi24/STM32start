
#include "main.h"

#include "core.h"
#include "curve.h"
#include "adc.h"
#include "dmx_receiver.h"
#include "led.h"
#include "my9291.h"
//-h24 #include "usb_debug.h"
#include "dbg.h" //+h24

#define ADC_CHANNELS 		2
#define ADC_MAX 		1800

#define ADDR_LED_RGBW	1

//-h24 extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

static uint8_t packet[576];
/*
static uint16_t adc[ADC_CHANNELS];

static void process_adc(uint16_t *data)
{
	uint8_t ch;

	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

	for (ch = 0; ch < ADC_CHANNELS; ch++)
		adc[ch] = data[ch];
}
*/
bool core_init(void)
{
/*-h24
	if (!adc_init(&hadc1, ADC_CHANNELS, process_adc))
		return false;
*/
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
/*-h24
	if ((adc[0] < ADC_MAX) && (adc[1] < ADC_MAX))
		usb_dumppacket(packet, len);
	else
//-h24		usb_printf("DMX.Len=%d ADC1=%d ADC2=%d\r\n", len, adc[0], adc[1]);
 */
	if(!HAL_GPIO_ReadPin(SW_BLUE_GPIO_Port, SW_BLUE_Pin)){
	      printf("/%3d/ %3d %3d %3d %3d\r\n", ADDR_LED_RGBW, packet[ADDR_LED_RGBW], packet[ADDR_LED_RGBW+1], packet[ADDR_LED_RGBW+2], packet[ADDR_LED_RGBW+3]);
		  dbg_dumppacket(packet, len);

	}

}
