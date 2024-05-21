#ifndef MY92X_H
#define MY92X_H

//#include <stdint.h>
#include <core.h>

//#define DLY_8US   16+2
#define DLY_12US  30//26
//#define DLY_16US  36+2


//#define DEBUG_MY92XX
//#define DEBUG_MY92XX_DBG2

// Define constants for the model types
#define MY92XX_MODEL_MY9291 0x00
#define MY92XX_MODEL_MY9231 0x01

// Define constants for command types
#define MY92XX_CMD_ONE_SHOT_DISABLE 0x00
#define MY92XX_CMD_ONE_SHOT_ENFORCE 0x01

#define MY92XX_CMD_REACTION_FAST 0x00
#define MY92XX_CMD_REACTION_SLOW 0x01

#define MY92XX_CMD_BIT_WIDTH_16 0x00
#define MY92XX_CMD_BIT_WIDTH_14 0x01
#define MY92XX_CMD_BIT_WIDTH_12 0x02
#define MY92XX_CMD_BIT_WIDTH_8 0x03

#define MY92XX_CMD_FREQUENCY_DIVIDE_1 0x00
#define MY92XX_CMD_FREQUENCY_DIVIDE_4 0x01
#define MY92XX_CMD_FREQUENCY_DIVIDE_16 0x02
#define MY92XX_CMD_FREQUENCY_DIVIDE_64 0x03

#define MY92XX_CMD_SCATTER_APDM 0x00
#define MY92XX_CMD_SCATTER_PWM 0x01

                            //   76543210
#define MY92XX_COMMAND_DEFAULT 0b00011000
                            // CMD[7] 0 ...temp
                            // CMD[6] 0 ...onest    MY92XX_CMD_ONE_SHOT_DISABLE
                            // CMD[5] 0 ...hspdb    MY92XX_CMD_REACTION_FAST
                            // CMD[4] 1 ...bs1
                            // CMD[3] 1 ...bs0      MY92XX_CMD_BIT_WIDTH_8
                            // CMD[2] 0 ...gck1
                            // CMD[1] 0 ...gck0     MY92XX_CMD_FREQUENCY_DIVIDE_1
                            // CMD[0] 0 ...sepb     MY92XX_CMD_SCATTER_APDM


// Define the structure to hold the my92xx command
typedef struct {
    uint8_t scatter:1;
    uint8_t frequency:2;
    uint8_t bit_width:2;
    uint8_t reaction:1;
    uint8_t one_shot:1;
    unsigned char resv:1;
} __attribute__ ((aligned(1), packed)) my92xx_cmd_t;
/*
#define MY92XX_COMMAND_DEFAULT { \
    .scatter = MY92XX_CMD_SCATTER_APDM, \
    .frequency = MY92XX_CMD_FREQUENCY_DIVIDE_1, \
    .bit_width = MY92XX_CMD_BIT_WIDTH_8, \
    .reaction = MY92XX_CMD_REACTION_FAST, \
    .one_shot = MY92XX_CMD_ONE_SHOT_DISABLE, \
    .resv = 0 \
}
*/

void my92xx_dly_us(uint16_t time);              //16=8.3us 26=12.2us 36=16.2us

void my92xx_di_pulse(uint16_t times);
void my92xx_dcki_pulse(uint16_t times);
void my92xx_write(uint16_t data, uint8_t bit_length);

void my92xx_write_h24(uint16_t data, uint8_t bit_length);


uint8_t my92xx_getChannels();
void my92xx_setChannel(uint8_t channel, uint16_t val);
uint16_t my92xx_getChannel(uint8_t channel);
void my92xx_setState(bool state);
bool my92xx_getState();
void my92xx_update();

void my92xx_set_cmd(uint8_t command);
void my92xx_send();

void my92xx_init_blue(void);
void my92xx_init(uint8_t model, uint8_t chips, uint8_t command);



#endif		/* MY92X_H */
