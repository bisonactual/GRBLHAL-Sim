/*
  eeprom.c - EEPROM/NVS persistence to file

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "eeprom.h"
#include "grbl/crc.h"
#include "grbl/hal.h"

#define EEPROM_SIZE 16384  // 16K to match FlexiHAL's 128 (16K) EEPROM

static uint8_t eeprom[EEPROM_SIZE];
static char eeprom_filename[256] = "EEPROM.DAT";
static bool dirty = false;

void set_eeprom_name(const char *name)
{
    strncpy(eeprom_filename, name, sizeof(eeprom_filename) - 1);
}

static void eeprom_load(void)
{
    FILE *f = fopen(eeprom_filename, "rb");
    if (f) {
        fread(eeprom, 1, EEPROM_SIZE, f);
        fclose(f);
    } else {
        memset(eeprom, 0xFF, EEPROM_SIZE);
    }
}

void eeprom_close(void)
{
    if (dirty) {
        FILE *f = fopen(eeprom_filename, "wb");
        if (f) {
            fwrite(eeprom, 1, EEPROM_SIZE, f);
            fclose(f);
        }
        dirty = false;
    }
}

uint8_t eeprom_get_char(uint32_t addr)
{
    static bool loaded = false;
    if (!loaded) {
        eeprom_load();
        loaded = true;
    }
    return addr < EEPROM_SIZE ? eeprom[addr] : 0xFF;
}

void eeprom_put_char(uint32_t addr, uint8_t new_value)
{
    if (addr < EEPROM_SIZE) {
        eeprom[addr] = new_value;
        dirty = true;
    }
}

bool memcpy_to_eeprom(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    uint32_t i;
    for (i = 0; i < size; i++)
        eeprom_put_char(destination + i, source[i]);

    if (with_checksum)
        eeprom_put_char(destination + size, calc_checksum(source, size));

    return true;
}

bool memcpy_from_eeprom(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    uint32_t i;
    for (i = 0; i < size; i++)
        destination[i] = eeprom_get_char(source + i);

    return true;
}
