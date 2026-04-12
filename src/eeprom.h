/*
  eeprom.h - EEPROM/NVS persistence for simulator

  Part of grblHAL FlexiHAL Simulator
*/

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdint.h>
#include <stdbool.h>

void set_eeprom_name(const char *name);
void eeprom_close(void);
uint8_t eeprom_get_char(uint32_t addr);
void eeprom_put_char(uint32_t addr, uint8_t new_value);
bool memcpy_to_eeprom(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum);
bool memcpy_from_eeprom(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum);

#endif
