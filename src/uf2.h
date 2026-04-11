/*
  uf2.h - UF2 firmware upload + plugin manager

  Part of grblHAL FlexiHAL Simulator
*/

#ifndef _UF2_H_
#define _UF2_H_

#include <stdint.h>
#include <stdbool.h>

// UF2 magic numbers
#define UF2_MAGIC_START0 0x0A324655
#define UF2_MAGIC_START1 0x9E5D5157
#define UF2_MAGIC_END    0x0AB16F30
#define UF2_FAMILY_STM32F4 0x57755a57

typedef struct {
    uint32_t magic_start0;
    uint32_t magic_start1;
    uint32_t flags;
    uint32_t target_addr;
    uint32_t payload_size;
    uint32_t block_no;
    uint32_t num_blocks;
    uint32_t family_id;
    uint8_t  data[476];
    uint32_t magic_end;
} uf2_block_t;

// Set the simulator root and build directory paths (call before init)
void uf2_set_paths(const char *root, const char *build);

// Initialize HTTP server on given port
int uf2_http_init(uint16_t port);

// Non-blocking poll — returns true if firmware was uploaded
bool uf2_poll(void);

// Returns true if a rebuild was completed and restart is needed
bool uf2_rebuild_requested(void);

// Cleanup
void uf2_shutdown(void);

#endif
