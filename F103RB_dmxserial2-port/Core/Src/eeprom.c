#include <stdint.h>
#include "eeprom.h"

uint16_t readEEPROMHalfWord(uint16_t addr) {
	printf("readEEPROMHalfWord();");
	return 1234;
}

void writeEEPROMHalfWord(uint8_t addr, uint8_t data) {
	printf("writeEEPROMHalfWord");
}

void enableEEPROMWriting(void) {
	printf("writeEEPROMHalfWord");

}

void disableEEPROMWriting(void) {
	printf("writeEEPROMHalfWord");

}
