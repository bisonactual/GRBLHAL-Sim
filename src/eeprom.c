/*
  eeprom.c - EEPROM/NVS persistence to file

  Part of grblHAL FlexiHAL Simulator

  Writes go straight to the file, in place, like the board's EEPROM/FRAM: settings
  survive the simulator being killed (a power cut), and a kill mid-write can
  touch only the bytes being written.

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

// Creates the whole file on the first write. If a write fails, dirty stays set.
static bool write_through(uint32_t addr, uint32_t size)
{
    FILE *f = fopen(eeprom_filename, "r+b");

    if (!f && (f = fopen(eeprom_filename, "w+b"))) {
        addr = 0;
        size = EEPROM_SIZE;
    }
    if (addr >= EEPROM_SIZE)
        size = 0;
    else if (size > EEPROM_SIZE - addr)
        size = EEPROM_SIZE - addr;

    bool ok = f && fseek(f, (long)addr, SEEK_SET) == 0 && fwrite(eeprom + addr, 1, size, f) == size;
    if (f && fclose(f) != 0)
        ok = false;
    if (ok)
        dirty = false;

    return ok;
}

void eeprom_close(void)
{
    if (dirty)
        write_through(0, EEPROM_SIZE);
}

static void ensure_loaded(void)
{
    static bool loaded = false;
    if (!loaded) {
        eeprom_load();
        loaded = true;
    }
}

uint8_t eeprom_get_char(uint32_t addr)
{
    ensure_loaded();
    return addr < EEPROM_SIZE ? eeprom[addr] : 0xFF;
}

static void put_char(uint32_t addr, uint8_t new_value)
{
    ensure_loaded();
    if (addr < EEPROM_SIZE) {
        eeprom[addr] = new_value;
        dirty = true;
    }
}

void eeprom_put_char(uint32_t addr, uint8_t new_value)
{
    put_char(addr, new_value);
    write_through(addr, 1);
}

bool memcpy_to_eeprom(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    uint32_t i;
    for (i = 0; i < size; i++)
        put_char(destination + i, source[i]);

    if (with_checksum)
        put_char(destination + size, calc_checksum(source, size));

    return write_through(destination, size + (with_checksum ? 1 : 0));
}

bool memcpy_from_eeprom(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    uint32_t i;
    for (i = 0; i < size; i++)
        destination[i] = eeprom_get_char(source + i);

    return true;
}
