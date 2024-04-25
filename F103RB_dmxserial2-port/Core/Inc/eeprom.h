// Fake EEPROM routines

#ifndef __EEPROM_H
#define __EEPROM_H

uint16_t readEEPROMHalfWord(uint16_t addr);
void writeEEPROMHalfWord(uint8_t addr, uint8_t data);



void enableEEPROMWriting(void);
void disableEEPROMWriting(void);


#endif /* __EEPROM_H */
