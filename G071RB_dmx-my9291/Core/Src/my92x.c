//https://github.com/xoseperez/my92xx/blob/master/src/my92xx.cpp

#include "my92x.h"

/*
 my92xx_model_t model = MY92XX_MODEL;
 uint8_t chips = MY92XX_CHIPS;
 uint8_t di_pin = MY92XX_DI_PIN;
 uint8_t dcki_pin = MY92XX_DCKI_PIN;
 uint8_t value[MY92XX_CHIPS * 4]; // Assuming 4 channels per chip for MY9291
 uint8_t channels = MY92XX_CHIPS * 4;
 my92xx_cmd_t command = MY92XX_COMMAND_DEFAULT;
 */
/*
 uint8_t _pin_di;
 uint8_t _pin_dcki;
 */
bool _state = false;
my92xx_cmd_t _command;
uint16_t *_value;
uint8_t _channels;
uint8_t _model = MY92XX_MODEL_MY9291;
uint8_t _chips = MY92XX_CHIPS;

uint16_t value[MY92XX_CHIPS * 4]; // Assuming 4 channels per chip for MY9291

void my92xx_dly_us(uint16_t time) {             // 16=8.3us, 26=12.2us 36=16.2us
    for (uint16_t i = 0; i < time; i++) {       // DLY_8US,  DLY_12US, DLY_16US
        asm("NOP");
    }
}

void my92xx_di_pulse(uint16_t times) {
    for (uint16_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);
    }
}

void my92xx_dcki_pulse(uint16_t times) {
    for (uint16_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);
    }
}

void my92xx_write_h24(uint16_t data, uint8_t bit_length) {
    uint16_t mask = (0x01 << (bit_length - 1));

    //12us together
    my92xx_dly_us(DLY_12US);

    //1 DI pulse h24
    LED_DI_GPIO_Port->BSRR = (uint32_t) LED_DI_Pin; //SET
    LED_DI_GPIO_Port->BRR = (uint32_t) LED_DI_Pin;  //RESET

    my92xx_dly_us(DLY_12US);


    for (uint16_t i = 0; i < bit_length / 2; i++) {
        LED_DCKI_GPIO_Port->BRR = (uint32_t) LED_DCKI_Pin;  //RESET

        if (data & mask) {
            LED_DI_GPIO_Port->BSRR = (uint32_t) LED_DI_Pin; //SET
        } else {
            LED_DI_GPIO_Port->BRR = (uint32_t) LED_DI_Pin;  //RESET
        }
        data = data << 1;
        LED_DCKI_GPIO_Port->BSRR = (uint32_t) LED_DCKI_Pin; //SET

        if (data & mask) {
            LED_DI_GPIO_Port->BSRR = (uint32_t) LED_DI_Pin; //SET
        } else {
            LED_DI_GPIO_Port->BRR = (uint32_t) LED_DI_Pin;  //RESET
        }
        data = data << 1;
        LED_DCKI_GPIO_Port->BRR = (uint32_t) LED_DCKI_Pin;  //RESET

        LED_DI_GPIO_Port->BRR = (uint32_t) LED_DI_Pin;  //RESET
    }
}

void my92xx_write(uint16_t data, uint8_t bit_length) {
    uint16_t mask = (0x01 << (bit_length - 1));

    my92xx_dly_us(DLY_12US);

    //1 DI pulse h24
    HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);

    my92xx_dly_us(DLY_12US);


    for (uint16_t i = 0; i < bit_length / 2; i++) {
        HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin,
                (data & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data = data << 1;
        HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_SET);


        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin,
                (data & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data = data << 1;
        HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);
    }
}

void my92xx_set_cmd(uint8_t command) {

#ifdef DEBUG_MY92XX
	printf("[MY92XX] Command: 0x%X \r\n", command);
#endif

    // ets_intr_lock();

    // TStop > 12us.
    my92xx_dly_us(DLY_12US);

    // Send 12 DI pulse, after 6 pulse's falling edge store duty data, and 12
    // pulse's rising edge convert to command mode.
    my92xx_di_pulse(12);

    // Delay >12us, begin send CMD data
    my92xx_dly_us(DLY_12US); //allready in my92xx_write_h24();


    // Send CMD data
    uint8_t command_data = *(uint8_t*) (&command);
    for (uint8_t i = 0; i < _chips; i++) {
        //my92xx_write(command_data, 8);
    	my92xx_write_h24(command_data, 8);
    }

    // TStart > 12us. Delay 12 us.
    ///os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // Send 16 DI pulse，at 14 pulse's falling edge store CMD data, and
    // at 16 pulse's falling edge convert to duty mode.
    my92xx_di_pulse(16);

    // TStop > 12us.
    my92xx_dly_us(DLY_12US);

    // ets_intr_unlock();

}

void my92xx_send() {
    uint8_t bit_length = 8;
    /*
     uint8_t bit_width = MY92XX_CMD_BIT_WIDTH_8;

     switch (bit_width) {
     case MY92XX_CMD_BIT_WIDTH_16:
     bit_length = 16;
     break;
     case MY92XX_CMD_BIT_WIDTH_14:
     bit_length = 14;
     break;
     case MY92XX_CMD_BIT_WIDTH_12:
     bit_length = 12;
     break;
     case MY92XX_CMD_BIT_WIDTH_8:
     bit_length = 8;
     break;
     default:
     bit_length = 8;
     break;
     }
     */
    _channels = 16;
#ifdef DEBUG_MY92XX
	printf("[MY92XX] Send...\r\n");
	printf("_chanels   = %d\r\n", _channels);
	printf("bit_length = %d\r\n", bit_length);

	printf("[MY92XX] Send: %s (", _state ? "ON" : "OFF");
	for (uint8_t channel = 0; channel < _channels; channel++) {
		printf(" %d", value[channel]);
	}
	printf(" )\r\n");
#endif

    // ets_intr_lock();


    // TStop > 12us.
    //os_delay_us(12);
    my92xx_dly_us(DLY_12US); //allready in my92xx_write_h24();


    // Send color data
    for (uint8_t channel = 0; channel < _channels; channel++) {
        //my92xx_write(_state ? value[channel] : 0, bit_length);
        my92xx_write_h24(_state ? value[channel] : 0, bit_length);
    }

    // TStart > 12us. Ready for send DI pulse.
    //os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // Send 8 DI pulse. After 8 pulse falling edge, store old data.
    //_di_pulse(8);
    my92xx_di_pulse(8);

    // TStop > 12us.
    //os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // ets_intr_unlock();
}


// -----------------------------------------------------------------------------

unsigned char my92xx_getChannels() {
    return _channels;
}

void my92xx_setChannel(uint8_t channel, uint16_t val) {
    if (channel < _channels) {
        value[channel] = val;
    }
}

uint16_t my92xx_getChannel(uint8_t channel) {
    if (channel < _channels) {
        return value[channel];
    }
    return 0;
}

bool my92xx_getState() {
    return _state;
}

void my92xx_setState(bool state) {
    _state = state;
}

void my92xx_update() {

    my92xx_send();

}

// -----------------------------------------------------------------------------
void my92xx_init_blue(void) {
	//	for (uint8_t i = 0; i < MY92XX_CHIPS; i++)
	//		my92xx_setChannel(i, 0);
	     my92xx_setChannel(MY92XX_B1, LED_RGBW_MY92_ON);
	     my92xx_setChannel(MY92XX_B2, LED_RGBW_MY92_ON);
	     my92xx_setChannel(MY92XX_B3, LED_RGBW_MY92_ON);
	     my92xx_setChannel(MY92XX_B4, LED_RGBW_MY92_ON);
	     my92xx_update();

}

void my92xx_init(uint8_t model, uint8_t chips, uint8_t command) {
    //uint8_t _channels;
    _model = model;
    _chips = chips;

    if (_model == MY92XX_MODEL_MY9291) {
        _channels = 4 * _chips;
    } else if (_model == MY92XX_MODEL_MY9231) {
        _channels = 3 * _chips;
    }
    for (uint8_t i = 0; i < _channels; i++) {
        value[i] = 0;
    }

#ifdef DEBUG_MY92XX
	printf("[MY92XX] Init...\r\n");
	printf("_model   = %d\r\n", _model);
	printf("_chips   = %d\r\n", _chips);
	printf("_chanels = %d\r\n", _channels);
	printf("command  = %X\r\n", command);

	printf("[MY92XX] Init: %s (", _state ? "ON" : "OFF");
	for (uint8_t channel = 0; channel < _channels; channel++) {
		printf(" %d", value[channel]);
	}
	printf(" )\r\n");
#endif

    HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);

    //DBG_OUT1_H();  DBG_OUT1_L();    //**//**//

// Clear all duty register
    my92xx_dcki_pulse(32 * _chips); //64*N/2 pulses after poweron

// Send init command
    my92xx_set_cmd(command);
    my92xx_setState(true);

//printf("[MY92XX] Initialized\r\n");

}
