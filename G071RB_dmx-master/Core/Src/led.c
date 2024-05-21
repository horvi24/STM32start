#include "led.h"

static TIM_HandleTypeDef *htim3;

void led_set(uint8_t led, uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;//*h24 TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (led == 0) {
		HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1);
	}
	if (led == 1) {
		HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_2);
	}
	if (led == 2) {
		HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_3);
	}
	if (led == 3) {
		HAL_TIM_PWM_ConfigChannel(htim3, &sConfigOC, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_4);
	}
}

bool led_init(TIM_HandleTypeDef *htim)
{
	TIM_OC_InitTypeDef sConfigOC;

	htim3 = htim;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
    sConfigOC.Pulse = 240;
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);

	return true;
}
