#ifndef DBG_H
#define DBG_H

#include "main.h"

//#define DBG_OUT1 //DBG pin PD9
//#define DBG_OUT2 //DBG pin PD8
//#define DBG_OUT3 //DBG pin PB13
//#define DBG_OUT4 //DBG pin PB10
//#define DBG_OUT5 //DBG pin PB15

#define OPT_SW1()       !HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)
#define OPT_SW2()       !HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)
#define OPT_SW3()       !HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)
#define OPT_SW4()       !HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin)

#define LED_HB_ON()    HAL_GPIO_WritePin(LED_HB_GPIO_Port, LED_HB_Pin, GPIO_PIN_SET)
#define LED_HB_OFF()   HAL_GPIO_WritePin(LED_HB_GPIO_Port, LED_HB_Pin, GPIO_PIN_RESET)


#define DEBUG_OUT_ENABLE

#ifdef DEBUG_OUT_ENABLE
#define	DBG_OUT1_H()  	HAL_GPIO_WritePin(DBG_OUT1_GPIO_Port, DBG_OUT1_Pin, GPIO_PIN_SET)
#define	DBG_OUT1_L() 	HAL_GPIO_WritePin(DBG_OUT1_GPIO_Port, DBG_OUT1_Pin, GPIO_PIN_RESET)
#define	DBG_OUT1()		DBG_OUT1_H(); DBG_OUT1_L()
#define	DBG_OUT2_H() 	HAL_GPIO_WritePin(DBG_OUT2_GPIO_Port, DBG_OUT2_Pin, GPIO_PIN_SET)
#define	DBG_OUT2_L() 	HAL_GPIO_WritePin(DBG_OUT2_GPIO_Port, DBG_OUT2_Pin, GPIO_PIN_RESET)
#define	DBG_OUT2()		DBG_OUT2_H(); DBG_OUT2_L()
#define	DBG_OUT3_H() 	HAL_GPIO_WritePin(DBG_OUT3_GPIO_Port, DBG_OUT3_Pin, GPIO_PIN_SET)
#define	DBG_OUT3_L() 	HAL_GPIO_WritePin(DBG_OUT3_GPIO_Port, DBG_OUT3_Pin, GPIO_PIN_RESET)
#define	DBG_OUT3()		DBG_OUT3_H(); DBG_OUT3_L()
#define	DBG_OUT4_H() 	HAL_GPIO_WritePin(DBG_OUT4_GPIO_Port, DBG_OUT4_Pin, GPIO_PIN_SET)
#define	DBG_OUT4_L() 	HAL_GPIO_WritePin(DBG_OUT4_GPIO_Port, DBG_OUT4_Pin, GPIO_PIN_RESET)
#define	DBG_OUT4()		DBG_OUT4_H(); DBG_OUT4_L()
#define	DBG_OUT5_H() 	HAL_GPIO_WritePin(DBG_OUT5_GPIO_Port, DBG_OUT5_Pin, GPIO_PIN_SET)
#define	DBG_OUT5_L() 	HAL_GPIO_WritePin(DBG_OUT5_GPIO_Port, DBG_OUT5_Pin, GPIO_PIN_RESET)
#define	DBG_OUT5()		DBG_OUT5_H(); DBG_OUT5_L()
#else
#define	DBG_OUT1_H()
#define	DBG_OUT1_L()
#define	DBG_OUT1()
#define	DBG_OUT2_H()
#define	DBG_OUT2_L()
#define	DBG_OUT2()
#define	DBG_OUT3_H()
#define	DBG_OUT3_L()
#define	DBG_OUT3()
#define	DBG_OUT4_H()
#define	DBG_OUT4_L()
#define	DBG_OUT4()
#define	DBG_OUT5_H()
#define	DBG_OUT5_L()
#define	DBG_OUT5()
#endif

void print_half_byte(uint8_t byte);
void print_byte(uint8_t byte);
void print_int(uint16_t byte);

void dbg_dumppacket(uint8_t *src_packet, uint16_t len);



#endif		/* DBG_H */
