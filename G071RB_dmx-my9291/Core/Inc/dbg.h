#ifndef DBG_H
#define DBG_H

//#include <stdint.h>
//#include "stm32g0xx_hal.h"

//#include "stm32g0xx_hal.h"
//#include <stdio.h>
//#include <stdint.h>

#define LED_HB_ON()     HAL_GPIO_WritePin(LED_HB_GPIO_Port, LED_HB_Pin, GPIO_PIN_SET)
#define LED_HB_OFF()    HAL_GPIO_WritePin(LED_HB_GPIO_Port, LED_HB_Pin, GPIO_PIN_RESET)

#define KEY_PRESSED()   !HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)


//#define DEBUG_MY92XX
//#define DEBUG_DMX_DBG1
//#define DEBUG_DMX_DBG1_2
//#define DEBUG_DMX_DBG1_3
//#define DEBUG_DMX_PRINTF_4

//#define DEBUG_MY92XX_DBG2
//#define DEBUG_CORE_DBG2
//#define DEBUG_MY92XX_DBG2_INIT


//#define DBG_OUT1 //DBG pin PD9
//#define DBG_OUT2 //DBG pin PD8
//#define DBG_OUT3 //DBG pin PB13
//#define DBG_OUT4 //DBG pin PB10
//#define DBG_OUT5 //DBG pin PB15

#define DEBUG_OUT_ENABLE

#ifdef DEBUG_OUT_ENABLE
#define DBG_OUT1_H()    HAL_GPIO_WritePin(DBG_OUT1_GPIO_Port, DBG_OUT1_Pin, GPIO_PIN_SET)
#define DBG_OUT1_L()    HAL_GPIO_WritePin(DBG_OUT1_GPIO_Port, DBG_OUT1_Pin, GPIO_PIN_RESET)
#define DBG_OUT1()      DBG_OUT1_H(); DBG_OUT1_L()
#define DBG_OUT2_H()    HAL_GPIO_WritePin(DBG_OUT2_GPIO_Port, DBG_OUT2_Pin, GPIO_PIN_SET)
#define DBG_OUT2_L()    HAL_GPIO_WritePin(DBG_OUT2_GPIO_Port, DBG_OUT2_Pin, GPIO_PIN_RESET)
#define DBG_OUT2()      DBG_OUT2_H(); DBG_OUT2_L()
#define DBG_OUT3_H()    HAL_GPIO_WritePin(DBG_OUT3_GPIO_Port, DBG_OUT3_Pin, GPIO_PIN_SET)
#define DBG_OUT3_L()    HAL_GPIO_WritePin(DBG_OUT3_GPIO_Port, DBG_OUT3_Pin, GPIO_PIN_RESET)
#define DBG_OUT3()      DBG_OUT3_H(); DBG_OUT3_L()
#define DBG_OUT4_H()    HAL_GPIO_WritePin(DBG_OUT4_GPIO_Port, DBG_OUT4_Pin, GPIO_PIN_SET)
#define DBG_OUT4_L()    HAL_GPIO_WritePin(DBG_OUT4_GPIO_Port, DBG_OUT4_Pin, GPIO_PIN_RESET)
#define DBG_OUT4()      DBG_OUT4_H(); DBG_OUT4_L()
#define DBG_OUT5_H()    HAL_GPIO_WritePin(DBG_OUT5_GPIO_Port, DBG_OUT5_Pin, GPIO_PIN_SET)
#define DBG_OUT5_L()    HAL_GPIO_WritePin(DBG_OUT5_GPIO_Port, DBG_OUT5_Pin, GPIO_PIN_RESET)
#define DBG_OUT5()      DBG_OUT5_H(); DBG_OUT5_L()
#else
#define DBG_OUT1_H()
#define DBG_OUT1_L()
#define DBG_OUT1()
#define DBG_OUT2_H()
#define DBG_OUT2_L()
#define DBG_OUT2()
#define DBG_OUT3_H()
#define DBG_OUT3_L()
#define DBG_OUT3()
#define DBG_OUT4_H()
#define DBG_OUT4_L()
#define DBG_OUT4()
#define DBG_OUT5_H()
#define DBG_OUT5_L()
#define DBG_OUT5()
#endif

void print_half_byte(uint8_t byte);
void print_byte(uint8_t byte);
void print_int(uint16_t byte);

void dbg_dumppacket(uint8_t *src_packet, uint16_t len);
void RGBW_red(void);


#endif		/* DBG_H */
