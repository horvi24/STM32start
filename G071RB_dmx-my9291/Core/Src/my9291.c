#include "my9291.h"
#define DLY_8US   16
#define DLY_12US  26
#define DLY_16US  36

//https://github.com/xoseperez/my92xx/blob/master/src/my92xx.cpp

void dly_us(uint16_t time){				// 16=8.3us, 26=12.2us 36=16.2us
	for(uint16_t i=0; i<time; i++){		// DLY_8US,  DLY_12US, DLY_16US
		asm("NOP");
	}
}

void my92xx_di_pulse(uint16_t times) {
	for (uint16_t i = 0; i < times; i++) {
		//digitalWrite(_pin_di, HIGH);
		//digitalWrite(_pin_di, LOW);
		HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);
	}
}

void my92xx_dcki_pulse(uint16_t times) {
	for (uint16_t i = 0; i < times; i++) {
		//digitalWrite(_pin_dcki, HIGH);
		//digitalWrite(_pin_dcki, LOW);
		HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);
	}
}

void my92xx_write(uint16_t data, uint8_t bit_length) {

	uint16_t mask = (0x01 << (bit_length - 1));

    for (uint16_t i = 0; i < bit_length / 2; i++) {
        //digitalWrite(_pin_dcki, LOW);
		HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);
        //digitalWrite(_pin_di, (data & mask) ? HIGH : LOW);
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, (data & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        //digitalWrite(_pin_dcki, HIGH);
		HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_SET);
        data = data << 1;

        //digitalWrite(_pin_di, (data & mask) ? HIGH : LOW);
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, (data & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        //digitalWrite(_pin_dcki, LOW);
		HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);
        //digitalWrite(_pin_di, LOW);
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);
        data = data << 1;
    }

}
