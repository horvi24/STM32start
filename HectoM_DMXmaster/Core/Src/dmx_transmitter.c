//https://github.com/aleksandrgilfanov/stm32f4-dmx-transmitter
#include "main.h"

//#include "dmx_transmitter.h"

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

volatile static uint16_t slots_sent;
volatile static uint16_t slots_count;
volatile static const uint8_t *slots_ptr;

void dmx_send(const uint8_t *slots, uint16_t size)
{
	/* Setup global variables, they will be accessed from interrupts  */
	slots_ptr = slots;
	slots_sent = 0;
	slots_count = size;

	/*
	 * DMX512 packet starts with Reset Sequence. Reset Sequence starts with
	 * Break (low level), so set pin to output low.
	 */
	/* org. verzija, ker je open-drain izhod ni potreben tri-state
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DMX_TX_BREAK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(DMX_TX_BREAK_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(DMX_TX_BREAK_GPIO_Port, DMX_TX_BREAK_Pin, GPIO_PIN_RESET);
    */
	HAL_GPIO_WritePin(DMX_TX_BREAK_GPIO_Port, DMX_TX_BREAK_Pin, GPIO_PIN_RESET);

	/*
	 * Waiting for Break time, and everything further, is done through TIM3
	 * interrupt. At first interrupt (CC1) this timer changes pin to high
	 * for MarkAfterBreak. Then, in update interrupt, it configures pin into
	 * floating mode. Right after this, it sends first DMX byte 0x00, and
	 * turn on second timer for sending slots after MarkBetween slots.
	 */
	DBG_OUT2_H();
	/* So, enable this timer */
 	__HAL_TIM_ENABLE(&htim17);



	/* Wait until transmission of whole DMX packet is finished */
	while (slots_ptr != NULL) {};
}

/* Interrupt handler for sending next slot (from TIM16 interrupt) */
void dmx_slot(void)
{
	/*
	 * Each slot is sent in Update Interrupt of TIM16. So, period of TIM16 is
	 * time, needed for 11bits of slot (44us) and Mark Between Slots (0..1s)
	 */
    //DBG_OUT3_H();
    DBG_OUT2();
	if (slots_sent < slots_count){

		USART2->TDR = slots_ptr[slots_sent++];
	}
	else {
		/* Stop timer when all slots are sent */
		TIM16->CR1 &= ~(TIM_CR1_CEN);
		TIM16->CNT = 0;

		/* Indicate that transmission finished */
		slots_ptr = NULL;
	}
	//DBG_OUT3_L();

}

/* Interrupt handler for sending reset sequence (from TIM17 interrupt) */
void dmx_reset_sequence(void)
{
    //DBG_OUT2();
	if (TIM17->SR & TIM_IT_UPDATE)						/*!<Update interrupt enable */
	{
		/* Reset sequence finished, stop timer */
		TIM17->CR1 &= ~(TIM_CR1_CEN);                  	/*!<Counter enable */
		TIM17->CNT = 0;
        DBG_OUT3_L();

		/* Send start code 0x00 */
		USART2->TDR = 0x00;

		/* Start timer for sending slots */
		__HAL_TIM_ENABLE(&htim16);
	}
	else if (TIM17->SR & TIM_IT_CC1)
	{
	    DBG_OUT2_L();
		/* Break end, set floating ping mode for Mark After Break */
	    /* org. verzija, ker je open-drain izhod ni potreben tri-state
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = DMX_TX_BREAK_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		HAL_GPIO_Init(DMX_TX_BREAK_GPIO_Port, &GPIO_InitStruct);
		 */
		HAL_GPIO_WritePin(DMX_TX_BREAK_GPIO_Port, DMX_TX_BREAK_Pin, GPIO_PIN_SET);
        DBG_OUT3_H();
	}
    //DBG_OUT2_L();
    //DBG_OUT3_L();

}
