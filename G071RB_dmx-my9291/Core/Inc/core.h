#ifndef CORE_H
#define CORE_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "curve.h"
#include "adc.h"
#include "dmx_receiver.h"
#include "led.h"
#include "my92x.h"
//-h24 #include "usb_debug.h"
#include "dbg.h" //+h24
/*
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_tim.h"
#include "stm32g0xx_hal_uart.h"
*/




#define CORE_TIMER_CLOCK	    48000000
#define CORE_PWM_FREQUENCY	    1200
#define CORE_PWM_TIMER_PERIOD 	 999
#define CORE_PWM_TIMER_PRESCALER (-1 + CORE_TIMER_CLOCK / \
				 (CORE_PWM_FREQUENCY * (CORE_PWM_TIMER_PERIOD + 1)))

#define ADDR_LED_RGBW_PWM	1
#define ADDR_LED_RGBW_MY92	1

#define LED_RGBW_MY92_ON    24


#define MY92XX_MODEL        MY92XX_MODEL_MY9291
#define MY92XX_CHIPS        4
//#define MY92XX_DI_PIN       13
//#define MY92XX_DCKI_PIN     15

#define MY92XX_R1		0
#define MY92XX_G1       1
#define MY92XX_B1       2
#define MY92XX_W1       3
#define MY92XX_R2		4
#define MY92XX_G2       5
#define MY92XX_B2       6
#define MY92XX_W2       7
#define MY92XX_R3		8
#define MY92XX_G3       9
#define MY92XX_B3       10
#define MY92XX_W3       11
#define MY92XX_R4		12
#define MY92XX_G4       13
#define MY92XX_B4       14
#define MY92XX_W4       15

#define RAINBOW_DELAY   10




bool core_init(void);
void core_process(void);

void core_process_h24(void);

#endif		/* CORE_H */
