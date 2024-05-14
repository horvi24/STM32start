#ifndef CORE_H
#define CORE_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "curve.h"
#include "adc.h"
#include "dmx_receiver.h"
#include "led.h"
#include "my92x.h"
//-h24 #include "usb_debug.h"
#include "dbg.h" //+h24




#define CORE_TIMER_CLOCK	 48000000
#define CORE_PWM_FREQUENCY	 1200
#define CORE_PWM_TIMER_PERIOD 	 999
#define CORE_PWM_TIMER_PRESCALER (-1 + CORE_TIMER_CLOCK / \
				 (CORE_PWM_FREQUENCY * (CORE_PWM_TIMER_PERIOD + 1)))


#define MY92XX_MODEL        MY92XX_MODEL_MY9291
#define MY92XX_CHIPS        4
//#define MY92XX_DI_PIN       13
//#define MY92XX_DCKI_PIN     15

#define MY92XX_RED          0
#define MY92XX_GREEN        1
#define MY92XX_BLUE         2
#define MY92XX_WHITE        4

#define RAINBOW_DELAY       10



bool core_init(void);
void core_process(void);

#endif		/* CORE_H */
