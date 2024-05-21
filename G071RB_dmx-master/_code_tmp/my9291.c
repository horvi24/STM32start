#include "my9291.h"

//https://github.com/xoseperez/my92xx/blob/master/src/my92xx.cpp

my92xx_cmd_t _command;
my92xx_model_t _model = MY92XX_MODEL_MY9291;
uint8_t _chips = 1;
uint8_t _channels;
uint16_t *_value;
bool _state = false;
uint8_t _pin_di;
uint8_t _pin_dcki;

void my92xx_dly_us(uint16_t time) {				// 16=8.3us, 26=12.2us 36=16.2us
    for (uint16_t i = 0; i < time; i++) {		// DLY_8US,  DLY_12US, DLY_16US
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
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin,
                (data & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        //digitalWrite(_pin_dcki, HIGH);
        HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_SET);
        data = data << 1;

        //digitalWrite(_pin_di, (data & mask) ? HIGH : LOW);
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin,
                (data & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        //digitalWrite(_pin_dcki, LOW);
        HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);
        //digitalWrite(_pin_di, LOW);
        HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);
        data = data << 1;
    }

}

void my92xx_set_cmd(my92xx_cmd_t command) {

    // ets_intr_lock();

    // TStop > 12us.
    ///os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // Send 12 DI pulse, after 6 pulse's falling edge store duty data, and 12
    // pulse's rising edge convert to command mode.
    ///_di_pulse(12);
    my92xx_di_pulse(12);

    // Delay >12us, begin send CMD data
    ///os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // Send CMD data
    uint8_t command_data = *(uint8_t*) (&command);
    for (uint8_t i = 0; i < _chips; i++) {
        my92xx_write(command_data, 8);
    }

    // TStart > 12us. Delay 12 us.
    ///os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // Send 16 DI pulse，at 14 pulse's falling edge store CMD data, and
    // at 16 pulse's falling edge convert to duty mode.
    ///_di_pulse(16);
    my92xx_di_pulse(16);

    // TStop > 12us.
    ///os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // ets_intr_unlock();

}

void my92xx_send() {

#ifdef DEBUG_MY92XX
        DEBUG_MSG_MY92XX("[MY92XX] Refresh: %s (", _state ? "ON" : "OFF");
        for (uint8_t channel = 0; channel < _channels; channel++) {
            //DEBUG_MSG_MY92XX(" %d", _value[channel]);
            printf(" %d", _value[channel]);
        }
        //DEBUG_MSG_MY92XX(" )\n");
        printf(" )\n");
    #endif

    uint8_t bit_length = 8;
    switch (_command.bit_width) {
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

    // ets_intr_lock();

    // TStop > 12us.
    //os_delay_us(12);
    my92xx_dly_us(DLY_12US);

    // Send color data
    for (uint8_t channel = 0; channel < _channels; channel++) {
        my92xx_write(_state ? _value[channel] : 0, bit_length);
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

uint8_t my92xx_getChannels() {
    return _channels;
}

void my92xx_setChannel(uint8_t channel, uint16_t value) {
    if (channel < _channels) {
        _value[channel] = value;
    }
}

uint16_t my92xx_getChannel(uint8_t channel) {
    if (channel < _channels) {
        return _value[channel];
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

uint16_t *_value;
uint8_t _channels;

_value = new uint16_t[_channels];

void my92xx_my92xx(my92xx_model_t model, uint8_t chips, uint8_t di,
        uint8_t dcki, my92xx_cmd_t command) {

    _model = model;
    _chips = chips;
    _pin_di = di;
    _pin_dcki = dcki;

    // Init channels
    if (_model == MY92XX_MODEL_MY9291) {
        _channels = 4 * _chips;
    } else if (_model == MY92XX_MODEL_MY9231) {
        _channels = 3 * _chips;
    }
    /*
     _value = new uint16_t[_channels];
     for (uint8_t i = 0; i < _channels; i++) {
     _value[i] = 0;
     }
     */
    _value = (uint16_t*) malloc(_channels * sizeof(uint16_t));
    if (_value == NULL) {
        // Handle memory allocation failure
        printf("Memory allocation failed\n");
    } else {
        // Memory allocation succeeded
        for (uint8_t i = 0; i < _channels; i++) {
            _value[i] = 0;
        }
    }

    // Init GPIO
    //pinMode(_pin_di, OUTPUT);
    //pinMode(_pin_dcki, OUTPUT);

    //digitalWrite(_pin_di, LOW);
    HAL_GPIO_WritePin(LED_DI_GPIO_Port, LED_DI_Pin, GPIO_PIN_RESET);
    //digitalWrite(_pin_dcki, LOW);
    HAL_GPIO_WritePin(LED_DCKI_GPIO_Port, LED_DCKI_Pin, GPIO_PIN_RESET);
    //varjanta  HAL_GPIO_WritePin(GPIOB, LED_DCKI_Pin|LED_DI_Pin, GPIO_PIN_RESET);

    // Clear all duty register
    my92xx_dcki_pulse(32 * _chips);

    // Send init command
    my92xx_set_cmd(command);

    //DEBUG_MSG_MY92XX("[MY92XX] Initialized\n");
    printf("[MY92XX] Initialized\n");
}

