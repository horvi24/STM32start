#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_tim.h"

bool led_init(TIM_HandleTypeDef *htim);
void led_set(uint8_t led, uint16_t value);

#endif		/* LED_H */
