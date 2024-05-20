//#include "main.h"

#include "core.h"

#define ADC_CHANNELS 		2
#define ADC_MAX 		1800

#define ADDR_LED_RGBW	1

//-h24 extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
/*
extern uint8_t Packet[DMX_MAX_SLOTS + 1];
extern uint8_t PacketFlag;
extern uint16_t PacketLength;
*/
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
bool core_init(void) {
	/*-h24
	 if (!adc_init(&hadc1, ADC_CHANNELS, process_adc))
	 return false;
	 */
	if (!led_init(&htim3))	//+h24
		return false;

	return true;
}


void core_process_h24(void) {
	uint16_t len;

	len = dmx_receive_24(packet);


	if (len == 0) {
		my92xx_update();
	}
	else {
		#ifdef DEBUG_CORE_DBG2
			HAL_GPIO_WritePin(DBG_OUT2_GPIO_Port, DBG_OUT2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DBG_OUT2_GPIO_Port, DBG_OUT2_Pin, GPIO_PIN_RESET);
		#endif

		led_set(0, curve_fn(packet[ADDR_LED_RGBW_PWM]));
		led_set(1, curve_fn(packet[ADDR_LED_RGBW_PWM + 1]));
		led_set(2, curve_fn(packet[ADDR_LED_RGBW_PWM + 2]));
		led_set(3, curve_fn(packet[ADDR_LED_RGBW_PWM + 3]));

		my92xx_setChannel(MY92XX_R1, packet[ADDR_LED_RGBW_MY92]);
		my92xx_setChannel(MY92XX_G1, packet[ADDR_LED_RGBW_MY92 + 1]);
		my92xx_setChannel(MY92XX_B1, packet[ADDR_LED_RGBW_MY92 + 2]);
		my92xx_setChannel(MY92XX_W1, packet[ADDR_LED_RGBW_MY92 + 3]);

		my92xx_setChannel(MY92XX_R2, packet[ADDR_LED_RGBW_MY92+4]);
		my92xx_setChannel(MY92XX_G2, packet[ADDR_LED_RGBW_MY92+5]);
		my92xx_setChannel(MY92XX_B2, packet[ADDR_LED_RGBW_MY92+6]);
		my92xx_setChannel(MY92XX_W2, packet[ADDR_LED_RGBW_MY92+7]);

		my92xx_setChannel(MY92XX_R3, packet[ADDR_LED_RGBW_MY92+8]);
		my92xx_setChannel(MY92XX_G3, packet[ADDR_LED_RGBW_MY92+9]);
		my92xx_setChannel(MY92XX_B3, packet[ADDR_LED_RGBW_MY92+10]);
		my92xx_setChannel(MY92XX_W3, packet[ADDR_LED_RGBW_MY92+11]);

		my92xx_setChannel(MY92XX_R4, packet[ADDR_LED_RGBW_MY92+12]);
		my92xx_setChannel(MY92XX_G4, packet[ADDR_LED_RGBW_MY92+13]);
		my92xx_setChannel(MY92XX_B4, packet[ADDR_LED_RGBW_MY92+14]);
		my92xx_setChannel(MY92XX_W4, packet[ADDR_LED_RGBW_MY92+15]);

		//my92xx_init(MY92XX_MODEL, MY92XX_CHIPS, MY92XX_COMMAND_DEFAULT);   //
		my92xx_update();

		if (!HAL_GPIO_ReadPin(SW_BLUE_GPIO_Port, SW_BLUE_Pin)) {
			printf("/%3d/ %3d %3d %3d %3d\r\n", ADDR_LED_RGBW,
					packet[ADDR_LED_RGBW], packet[ADDR_LED_RGBW + 1],
					packet[ADDR_LED_RGBW + 2], packet[ADDR_LED_RGBW + 3]);
			dbg_dumppacket(packet, len);

		}


	}


}

void core_process(void) {
	uint16_t len;

	len = dmx_receive(packet);

	led_set(0, curve_fn(packet[ADDR_LED_RGBW_PWM]));
	led_set(1, curve_fn(packet[ADDR_LED_RGBW_PWM + 1]));
	led_set(2, curve_fn(packet[ADDR_LED_RGBW_PWM + 2]));
	led_set(3, curve_fn(packet[ADDR_LED_RGBW_PWM + 3]));

	my92xx_setChannel(MY92XX_R1, packet[ADDR_LED_RGBW_MY92]);
	my92xx_setChannel(MY92XX_G1, packet[ADDR_LED_RGBW_MY92 + 1]);
	my92xx_setChannel(MY92XX_B1, packet[ADDR_LED_RGBW_MY92 + 2]);
	my92xx_setChannel(MY92XX_W1, packet[ADDR_LED_RGBW_MY92 + 3]);

	my92xx_update();

	if (!HAL_GPIO_ReadPin(SW_BLUE_GPIO_Port, SW_BLUE_Pin)) {
		printf("/%3d/ %3d %3d %3d %3d\r\n", ADDR_LED_RGBW,
				packet[ADDR_LED_RGBW], packet[ADDR_LED_RGBW + 1],
				packet[ADDR_LED_RGBW + 2], packet[ADDR_LED_RGBW + 3]);
		dbg_dumppacket(packet, len);

	}

}
