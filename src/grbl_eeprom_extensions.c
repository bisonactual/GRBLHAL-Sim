/*
  grbl_eeprom_extensions.c

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io
*/

#include <stdint.h>
#include "grbl_eeprom_extensions.h"

uint8_t calc_checksum(uint8_t *data, uint32_t size)
{
    uint8_t checksum = 0;
    while (size--)
        checksum = (checksum << 1) | (checksum >> 7);
    return checksum;
}
