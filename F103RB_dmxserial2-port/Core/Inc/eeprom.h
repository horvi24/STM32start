// Fake EEPROM routines

#ifndef __EEPROM_H
#define __EEPROM_H

uint16_t readEEPROMHalfWord(uint16_t addr);
uint16_t writeEEPROMHalfWord(uint16_t addr, uint16_t data);


void enableEEPROMWriting(void);
void disableEEPROMWriting(void);


#endif /* __MAIN_H */
