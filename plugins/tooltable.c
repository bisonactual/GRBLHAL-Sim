/*

  tooltable.c - file based tooltable, LinuxCNC format

  Part of grblHAL

  Copyright (c) 2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if TOOLTABLE_ENABLE == 2

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#if SDCARD_ENABLE
/* stripped: #include "sdcard/sdcard.h" (not found in plugins/) */
#endif

#include "grbl/vfs.h"
#include "grbl/strutils.h"
#include "grbl/gcode.h"
#include "grbl/stream.h"
#include "grbl/core_handlers.h"
#include "grbl/state_machine.h"

/* ---- inlined from tooltable.h ---- */
/*
  tooltable.h - file based tooltable, LinuxCNC format

  Part of grblHAL

  Copyright (c) 2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "grbl/gcode.h"

// Result codes for carousel operations
typedef enum {
    CarouselOp_OK = 0,
    CarouselOp_AlreadyRegistered,   // tool already at P0, no change made
    CarouselOp_ToolNotFound,        // tool_id not in tooltable
    CarouselOp_ToolAlreadyInPocket, // tool already has a pocket assigned
    CarouselOp_NoPocketAvailable,   // carousel is full
    CarouselOp_WriteError,          // tooltable file write failed
    CarouselOp_TableNotLoaded       // tooltable not yet loaded
} carousel_op_result_t;

void tooltable_set_max_pockets (uint16_t n);

// Return the name/comment string for a tool from the RAM index.
// Returns NULL if the tool is not indexed or has no name.
const char *tooltable_get_name (tool_id_t tool_id);

// Register a tool in the tooltable at P0 (not in the carousel).
// If the tool already exists its name is updated if name is non-NULL.
// If the tool already has a pocket assigned, returns CarouselOp_ToolAlreadyInPocket.
carousel_op_result_t tooltable_register_tool (tool_id_t tool_id, const char *name);

// Add a tool to the carousel.
// Finds the lowest-numbered free pocket, assigns the tool to it, and
// persists the change to the tooltable file.
// Returns CarouselOp_OK on success, or an error code otherwise.
// On success, *assigned_pocket (if non-NULL) is set to the pocket number assigned.
carousel_op_result_t tooltable_carousel_add (tool_id_t tool_id, uint16_t max_pockets, const char *name, pocket_id_t *assigned_pocket);

// Remove a tool from the carousel (clears pocket_id only — offsets persist)
// and persists the change to the tooltable file.
// Returns CarouselOp_OK on success, or an error code otherwise.
carousel_op_result_t tooltable_carousel_remove (tool_id_t tool_id);

// Delete a P0 tool entry from the tooltable entirely.
// Only tools with no carousel pocket assignment may be deleted — returns
// CarouselOp_ToolAlreadyInPocket if the tool is currently in a pocket.
// Returns CarouselOp_OK on success, or an error code otherwise.
carousel_op_result_t tooltable_delete (tool_id_t tool_id);
/* ---- end tooltable.h ---- */

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static bool              fs_available = false;  // VFS has been mounted
static tool_id_t         current_tool = 0;      // tool currently in spindle
static char              filename[]   = "/tooltable.tbl";

static uint16_t    max_pockets = 0;   // set by ATC plugin via tooltable_set_max_pockets()

// Zeroed fallback pocket — always valid, used before FS mounts or on empty table.
// Mirrors the pocket0 pattern from the TOOLTABLE_ENABLE==1 implementation.
static tool_pocket_t     pocket0      = {0};

static tool_select_ptr       tool_select;
static on_tool_changed_ptr   on_tool_changed;
static on_vfs_mount_ptr      on_vfs_mount;
static on_report_options_ptr on_report_options;

static tool_data_t      pending_set_tool     = {0};
static bool             pending_set_tool_valid = false;
static on_macro_return_ptr on_set_return   = NULL;

void tooltable_set_max_pockets (uint16_t n)
{
    max_pockets = n;
}

// ---------------------------------------------------------------------------
// Read one line from the file into buf. Returns true if a line was read.
// ---------------------------------------------------------------------------
static bool read_line (vfs_file_t *file, char *buf, size_t len)
{
    size_t idx = 0;
    char c;

    while(vfs_read(&c, 1, 1, file) == 1) {
        if(c == '\n') {
            buf[idx] = '\0';
            if(idx > 0 && buf[idx-1] == '\r')
                buf[idx-1] = '\0';
            return true;
        }
        if(idx < len - 1)
            buf[idx++] = c;
    }

    if(idx > 0) {
        buf[idx] = '\0';
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Parse one line into a tool_pocket_t.
// Returns true if a valid tool entry was parsed.
// ---------------------------------------------------------------------------
static bool parse_line (char *line, tool_pocket_t *out)
{
    if(!line || !*line || *line == ';' || *line == '\r' || *line == '\n')
        return false;

    memset(out, 0, sizeof(tool_pocket_t));
    out->pocket_id    = -1;
    out->tool.tool_id = -1;

    char *param = strtok(line, " \t");
    status_code_t status = Status_OK;

    while(param && status == Status_OK) {

        uint_fast8_t cc = 1;

        switch(CAPS(*param)) {

            case 'T':
            {
                uint32_t tool_id;
                if((status = read_uint(param, &cc, &tool_id)) == Status_OK)
                    out->tool.tool_id = (tool_id_t)tool_id;
            }
            break;

            case 'P':
            {
                uint32_t pocket_id;
                if((status = read_uint(param, &cc, &pocket_id)) == Status_OK)
                    out->pocket_id = (pocket_id == 0) ? -1 : (pocket_id_t)pocket_id;
            }
            break;

            case 'X':
                if(!read_float(param, &cc, &out->tool.offset.values[X_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;

            case 'Y':
                if(!read_float(param, &cc, &out->tool.offset.values[Y_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;

            case 'Z':
                if(!read_float(param, &cc, &out->tool.offset.values[Z_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#ifdef A_AXIS
            case 'A':
                if(!read_float(param, &cc, &out->tool.offset.values[A_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef B_AXIS
            case 'B':
                if(!read_float(param, &cc, &out->tool.offset.values[B_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef C_AXIS
            case 'C':
                if(!read_float(param, &cc, &out->tool.offset.values[C_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef U_AXIS
            case 'U':
                if(!read_float(param, &cc, &out->tool.offset.values[U_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
#ifdef V_AXIS
            case 'V':
                if(!read_float(param, &cc, &out->tool.offset.values[V_AXIS]))
                    status = Status_GcodeValueOutOfRange;
                break;
#endif
            case 'D':
                if(!read_float(param, &cc, &out->tool.radius))
                    status = Status_GcodeValueOutOfRange;
                else
                    out->tool.radius /= 2.0f;
                break;

            case ';':
                strncpy(out->name, param + 1, sizeof(out->name) - 1);
                while((param = strtok(NULL, " \t"))) {
                    if(strlen(out->name) + strlen(param) + 1 <= sizeof(out->name) - 1) {
                        if(*out->name)
                            strcat(out->name, " ");
                        strcat(out->name, param);
                    }
                }
                while((param = strchr(out->name, '|')))
                    *param = '%';
                param = NULL;
                break;
        }

        if(param)
            param = strtok(NULL, " \t");
    }

    return status == Status_OK && out->tool.tool_id >= 0;
}

// ---------------------------------------------------------------------------
// File scan — find a tool entry by tool_id.
// Scans the file linearly. Returns true and populates *out if found.
// Pass NULL for out if you only need to test existence.
// ---------------------------------------------------------------------------
static bool file_find (tool_id_t tool_id, tool_pocket_t *out)
{
    vfs_file_t *file = vfs_open(filename, "r");
    if(!file)
        return false;

    char line[300];
    tool_pocket_t entry;
    bool found = false;

    while(read_line(file, line, sizeof(line))) {
        if(parse_line(line, &entry) && entry.tool.tool_id == tool_id) {
            if(out)
                *out = entry;
            found = true;
            break;
        }
    }

    vfs_close(file);
/*
    char buf[120];
    if(found)
        sprintf(buf, "[file_find: T%ld P%d X%.3f Y%.3f Z%.3f D%.3f]\n",
                (long)tool_id,
                (int)(out ? out->pocket_id : entry.pocket_id),
                entry.tool.offset.values[X_AXIS],
                entry.tool.offset.values[Y_AXIS],
                entry.tool.offset.values[Z_AXIS],
                entry.tool.radius * 2.0f);
    else
        sprintf(buf, "[file_find: T%ld not found]\n", (long)tool_id);
    hal.stream.write(buf);
*/
    return found;
}

// ---------------------------------------------------------------------------
// File scan — find the lowest free carousel pocket number.
// Scans the file to build a used-pocket bitmap, then returns the lowest
// pocket number from 1 to max_pockets not currently assigned.
// ---------------------------------------------------------------------------
static pocket_id_t file_find_free_pocket (uint16_t max_pockets)
{
    if(max_pockets == 0)
        return -1;

    // Use a simple boolean array on the stack — max_pockets is at most a few hundred.
    // 500 pockets = 500 bytes stack, acceptable.
    bool in_use[max_pockets + 1];
    memset(in_use, 0, sizeof(in_use));

    vfs_file_t *file = vfs_open(filename, "r");
    if(file) {
        char line[300];
        tool_pocket_t entry;
        while(read_line(file, line, sizeof(line))) {
            if(parse_line(line, &entry) && entry.pocket_id >= 1 &&
               entry.pocket_id <= (pocket_id_t)max_pockets)
                in_use[entry.pocket_id] = true;
        }
        vfs_close(file);
    }

    for(pocket_id_t candidate = 1; candidate <= (pocket_id_t)max_pockets; candidate++) {
        if(!in_use[candidate])
            return candidate;
    }

    return -1;
}


// ---------------------------------------------------------------------------
// Write one pocket entry as a line to an open file.
// ---------------------------------------------------------------------------
static void write_pocket_line (vfs_file_t *file, const tool_pocket_t *p)
{
    char buf[400], tmp[24];

    uint16_t file_pocket = (p->pocket_id >= 1) ? (uint16_t)p->pocket_id : 0;
    sprintf(buf, "P%u T%u", file_pocket, (uint16_t)p->tool.tool_id);

    for(uint_fast8_t axis = 0; axis < N_AXIS; axis++) {
        if(fabsf(p->tool.offset.values[axis]) > 0.0001f) {
            sprintf(tmp, " %s%.3f", axis_letter[axis], p->tool.offset.values[axis]);
            strcat(buf, tmp);
        }
    }

    if(p->tool.radius != 0.0f) {
        sprintf(tmp, " D%.3f", p->tool.radius * 2.0f);
        strcat(buf, tmp);
    }

    if(*p->name) {
        strcat(buf, " ;");
        strcat(buf, p->name);
    }

    strcat(buf, "\n");
    vfs_write(buf, strlen(buf), 1, file);
}

// ---------------------------------------------------------------------------
// Rewrite the entire file, optionally overriding pocket_ids for specific
// tools.  Uses a temp file to avoid any heap allocation — one line at a
// time is read from the source, modified if it matches an override, and
// written to the temp file.  The temp file is then renamed over the original.
// Stack usage: one line buffer (300 bytes) + one tool_pocket_t (~200 bytes).
// ---------------------------------------------------------------------------
#define MAX_OVERRIDES 2

typedef struct {
    tool_id_t   tool_id;
    pocket_id_t new_pocket_id;
    char        name[sizeof(((tool_pocket_t*)0)->name)];  // optional: empty = no change
    bool        delete_entry;                              // if true, omit this tool from rewritten file
} pocket_override_t;

static char filename_tmp[] = "/tooltable.tmp";

static bool rewrite_file (const pocket_override_t *overrides, uint8_t n_overrides)
{
    vfs_file_t *src = vfs_open(filename, "r");
    if(!src)
        return false;

    vfs_file_t *dst = vfs_open(filename_tmp, "w");
    if(!dst) {
        vfs_close(src);
        return false;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(src, line, sizeof(line))) {

        if(!parse_line(line, &entry)) {
            // Blank or comment line — skip (do not carry forward comments
            // since the first-pass counter bug is gone and we never write them)
            continue;
        }

        // Check if this tool has a pocket_id or name override
        bool skip = false;
        for(uint8_t oi = 0; oi < n_overrides; oi++) {
            if(entry.tool.tool_id == overrides[oi].tool_id) {
                if(overrides[oi].delete_entry) {
                    skip = true;
                } else {
                    entry.pocket_id = overrides[oi].new_pocket_id;
                    if(overrides[oi].name[0] != '\0')
                        strncpy(entry.name, overrides[oi].name, sizeof(entry.name) - 1);
                }
                break;
            }
        }

        if(!skip)
            write_pocket_line(dst, &entry);
    }

    vfs_close(src);
    vfs_close(dst);

    // FAT filesystems cannot rename over an existing file — delete first.
    vfs_unlink(filename);
    if(vfs_rename(filename_tmp, filename) != 0) {
        vfs_unlink(filename_tmp);
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Append a brand-new tool entry to the end of the file.
// Avoids a full read-modify-write for the common $TCADD new-tool case.
// ---------------------------------------------------------------------------
static bool append_tool (const tool_pocket_t *p)
{
    vfs_file_t *src = vfs_open(filename, "r");
    if(!src)
        return false;

    vfs_file_t *dst = vfs_open(filename_tmp, "w");
    if(!dst) {
        vfs_close(src);
        return false;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(src, line, sizeof(line))) {
        if(!parse_line(line, &entry))
            continue;
        write_pocket_line(dst, &entry);
    }

    write_pocket_line(dst, p);

    vfs_close(src);
    vfs_close(dst);

    vfs_unlink(filename);
    if(vfs_rename(filename_tmp, filename) != 0) {
        vfs_unlink(filename_tmp);
        return false;
    }

    return true;
}

#define TOOL_CACHE_SIZE 4  // current, pending, and a couple of spares

static tool_table_entry_t cache_result[TOOL_CACHE_SIZE]  = {0};
static tool_pocket_t      cache_entry[TOOL_CACHE_SIZE]   = {0};
static tool_data_t        cache_unknown[TOOL_CACHE_SIZE] = {0};

static tool_table_entry_t *getTool (tool_id_t tool_id)
{
    // Find existing slot for this tool_id, or evict the oldest
    static uint8_t next_slot = 0;
    int slot = -1;

    // Check pending deferred write first — most up-to-date offset
    if(pending_set_tool_valid && pending_set_tool.tool_id == tool_id) {
        // Use slot 0 for pending tool — stable pointer
        slot = 0;
        cache_result[slot] = (tool_table_entry_t){0};
        cache_entry[slot].tool    = pending_set_tool;
        cache_entry[slot].pocket_id = 0;
        cache_entry[slot].name[0] = '\0';
        cache_result[slot].data   = &cache_entry[slot].tool;
        cache_result[slot].pocket = 0;
        cache_result[slot].name   = NULL;
        return &cache_result[slot];
    }

    // First check if this tool_id is already cached
    for(int i = 0; i < TOOL_CACHE_SIZE; i++) {
        if(cache_result[i].data && cache_result[i].data->tool_id == tool_id) {
            slot = i;
            break;
        }
    }

    // Not found — evict next slot in round-robin order, but never evict
    // the slot currently backing gc_state.tool
    if(slot == -1) {
        uint8_t attempts = 0;
        do {
            slot = next_slot;
            next_slot = (next_slot + 1) % TOOL_CACHE_SIZE;
            attempts++;
        } while(attempts < TOOL_CACHE_SIZE && 
                gc_state.tool && 
                cache_entry[slot].tool.tool_id == gc_state.tool->tool_id);
    }

    cache_result[slot] = (tool_table_entry_t){0};

    if(file_find(tool_id, &cache_entry[slot])) {
        cache_result[slot].data   = &cache_entry[slot].tool;
        cache_result[slot].pocket = cache_entry[slot].pocket_id;
        cache_result[slot].name   = cache_entry[slot].name;
        return &cache_result[slot];
    }

    if(!fs_available) {
        cache_result[slot] = (tool_table_entry_t){0};
        cache_result[slot].data   = &pocket0.tool;
        cache_result[slot].pocket = pocket0.pocket_id;
        cache_result[slot].name   = pocket0.name;
        return &cache_result[slot];
    }   

    cache_unknown[slot] = (tool_data_t){0};
    cache_unknown[slot].tool_id = tool_id;
    cache_result[slot].data   = &cache_unknown[slot];
    cache_result[slot].pocket = 0;
    cache_result[slot].name   = NULL;
    return &cache_result[slot];
}

// ---------------------------------------------------------------------------
// grbl.tool_table.get_tool_by_idx - look up tool by pocket number.
// Scans the file for the tool assigned to the given pocket (1-based index).
// ---------------------------------------------------------------------------
static tool_table_entry_t *getToolByIdx (uint32_t idx)
{
    static tool_table_entry_t result = {0};
    static tool_pocket_t      scanned = {0};

    result = (tool_table_entry_t){0};

    vfs_file_t *file = vfs_open(filename, "r");
    if(!file) {
        result.data   = &pocket0.tool;
        result.pocket = pocket0.pocket_id;
        result.name   = pocket0.name;
        return &result;
    }

    char line[300];
    tool_pocket_t entry;
    bool found = false;

    while(read_line(file, line, sizeof(line))) {
        if(parse_line(line, &entry) && entry.pocket_id == (pocket_id_t)idx) {
            scanned = entry;
            result.data   = &scanned.tool;
            result.pocket = scanned.pocket_id;
            result.name   = scanned.name;
            found = true;
            break;
        }
    }
    vfs_close(file);

    if(!found) {
        result.data   = &pocket0.tool;
        result.pocket = pocket0.pocket_id;
        result.name   = pocket0.name;
    }

    return &result;
}

//deferred set tool.
static bool set_tool_write (tool_data_t *tool_data)
{
    tool_id_t tool_id = tool_data->tool_id;

    vfs_file_t *src = vfs_open(filename, "r");
    if(!src)
        return false;

    vfs_file_t *dst = vfs_open(filename_tmp, "w");
    if(!dst) {
        vfs_close(src);
        return false;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(src, line, sizeof(line))) {
        if(!parse_line(line, &entry))
            continue;
        if(entry.tool.tool_id == tool_id) {
            memcpy(entry.tool.offset.values, tool_data->offset.values, sizeof(tool_data->offset.values));
            entry.tool.radius = tool_data->radius;
        }
        write_pocket_line(dst, &entry);
    }

    vfs_close(src);
    vfs_close(dst);

    vfs_unlink(filename);
    if(vfs_rename(filename_tmp, filename) != 0) {
        vfs_unlink(filename_tmp);
        return false;
    }

    return true;
}

static void deferred_set_tool (void)
{
    grbl.on_macro_return = on_set_return;
    on_set_return = NULL;

    if(pending_set_tool_valid) {
        tool_pocket_t existing;
        if(file_find(pending_set_tool.tool_id, &existing)) {
            set_tool_write(&pending_set_tool);
        } else {
            tool_pocket_t newentry = {0};
            newentry.tool      = pending_set_tool;
            newentry.pocket_id = 0;
            append_tool(&newentry);
        }
        // Clear after write so subsequent getTool calls read from file
        pending_set_tool_valid = false;
        memset(&pending_set_tool, 0, sizeof(pending_set_tool));
    }

    if(grbl.on_macro_return)
        grbl.on_macro_return();
}


static bool setTool (tool_data_t *tool_data)
{
    if(!tool_data || tool_data->tool_id <= 0)
        return false;

    if(hal.stream.file != NULL) {
        // Macro is running — defer the write, serve from pending until written
        pending_set_tool       = *tool_data;
        pending_set_tool_valid = true;
        on_set_return          = grbl.on_macro_return;
        grbl.on_macro_return   = deferred_set_tool;
        return true;
    }

    return set_tool_write(tool_data);
}

// ---------------------------------------------------------------------------
// grbl.tool_table.clear - zero offsets for all tools (tools remain in table).
// Rewrites the file with all offsets zeroed.
// ---------------------------------------------------------------------------
static bool clearTools (void)
{
    if(!fs_available)
        return true;

    vfs_file_t *src = vfs_open(filename, "r");
    if(!src)
        return false;

    vfs_file_t *dst = vfs_open(filename_tmp, "w");
    if(!dst) {
        vfs_close(src);
        return false;
    }

    char line[300];
    tool_pocket_t entry;

    while(read_line(src, line, sizeof(line))) {
        if(!parse_line(line, &entry))
            continue;
        memset(&entry.tool.offset, 0, sizeof(coord_data_t));
        entry.tool.radius = 0.0f;
        write_pocket_line(dst, &entry);
    }

    vfs_close(src);
    vfs_close(dst);

    vfs_unlink(filename);
    if(vfs_rename(filename_tmp, filename) != 0) {
        vfs_unlink(filename_tmp);
        return false;
    }

    return true;
}

// Return the name/comment for a tool by scanning the file.
// Returns NULL if the tool is not found or has no name.
// Uses a static buffer — safe for single-call use (e.g. operator notification).
const char *tooltable_get_name (tool_id_t tool_id)
{
    static tool_pocket_t entry;
    if(!file_find(tool_id, &entry) || entry.name[0] == '\0')
        return NULL;
    return entry.name;
}

// Register a tool in the tooltable at P0 (not in the carousel).
// If the tool already exists its name is updated if name is non-NULL and non-empty.
// Returns CarouselOp_ToolAlreadyInPocket if the tool already has a pocket assigned —
// use $TCADD to move an existing P0 tool into the carousel instead.
carousel_op_result_t tooltable_register_tool (tool_id_t tool_id, const char *name)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_pocket_t existing;
    bool found = file_find(tool_id, &existing);

    if(found) {
        // Tool already exists — update name if one was provided, preserve pocket
        if(name && *name) {
            pocket_override_t ov = {0};
            ov.tool_id       = tool_id;
            ov.new_pocket_id = existing.pocket_id;
            strncpy(ov.name, name, sizeof(ov.name) - 1);
            ov.name[sizeof(ov.name) - 1] = '\0';
            if(!rewrite_file(&ov, 1))
                return CarouselOp_WriteError;
            return CarouselOp_OK;
        }
        // Tool already registered, no name provided — nothing to do
        return CarouselOp_AlreadyRegistered;
    }

    // Brand-new tool — append as P0
    tool_pocket_t newentry = {0};
    newentry.tool.tool_id = tool_id;
    newentry.pocket_id    = 0;
    if(name && *name) {
        strncpy(newentry.name, name, sizeof(newentry.name) - 1);
        newentry.name[sizeof(newentry.name) - 1] = '\0';
    }
    if(!append_tool(&newentry))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

// ---------------------------------------------------------------------------
// Public carousel API
// ---------------------------------------------------------------------------

carousel_op_result_t tooltable_carousel_add (tool_id_t tool_id, uint16_t max_pockets, const char *name, pocket_id_t *assigned_pocket)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_pocket_t existing;
    bool found = file_find(tool_id, &existing);

    if(found && existing.pocket_id >= 1)
        return CarouselOp_ToolAlreadyInPocket;

    pocket_id_t free_pocket = file_find_free_pocket(max_pockets);
    if(free_pocket < 0)
        return CarouselOp_NoPocketAvailable;

    if(found) {
        // Tool exists as P0 in file — update pocket and optionally name via rewrite
        pocket_override_t ov = {0};
        ov.tool_id       = tool_id;
        ov.new_pocket_id = free_pocket;
        if(name && *name) {
            strncpy(ov.name, name, sizeof(ov.name) - 1);
            ov.name[sizeof(ov.name) - 1] = '\0';
        }
        if(!rewrite_file(&ov, 1))
            return CarouselOp_WriteError;
    } else {
        // Brand-new tool — append entry with optional name
        tool_pocket_t newentry = {0};
        newentry.tool.tool_id = tool_id;
        newentry.pocket_id    = free_pocket;
        if(name && *name) {
            strncpy(newentry.name, name, sizeof(newentry.name) - 1);
            newentry.name[sizeof(newentry.name) - 1] = '\0';
        }
        if(!append_tool(&newentry))
            return CarouselOp_WriteError;
    }

    if(assigned_pocket)
        *assigned_pocket = free_pocket;

    return CarouselOp_OK;
}


carousel_op_result_t tooltable_carousel_remove (tool_id_t tool_id)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_pocket_t existing;
    if(!file_find(tool_id, &existing) || existing.pocket_id < 1)
        return CarouselOp_ToolNotFound;

    pocket_override_t ov = { .tool_id = tool_id, .new_pocket_id = 0 };
    if(!rewrite_file(&ov, 1))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

// ---------------------------------------------------------------------------
// tooltable_delete() — Remove a P0 tool entry from the tooltable entirely.
//
// Only tools with no carousel pocket assignment (P0) may be deleted.  If the
// tool is currently assigned to a pocket, returns CarouselOp_ToolAlreadyInPocket
// so the caller can report a clear error without touching the file.
// ---------------------------------------------------------------------------
carousel_op_result_t tooltable_delete (tool_id_t tool_id)
{
    if(!fs_available)
        return CarouselOp_TableNotLoaded;

    tool_pocket_t existing;
    if(!file_find(tool_id, &existing))
        return CarouselOp_ToolNotFound;

    if(existing.pocket_id >= 1)
        return CarouselOp_ToolAlreadyInPocket;

    pocket_override_t ov = { .tool_id = tool_id, .delete_entry = true };
    if(!rewrite_file(&ov, 1))
        return CarouselOp_WriteError;

    return CarouselOp_OK;
}

// ---------------------------------------------------------------------------
// onToolChanged - called after M6/M61 completes.
// Registers new tools automatically and updates current_tool.
// Pocket assignments are permanent — managed explicitly via $TCADD/$TCRM.
// ---------------------------------------------------------------------------
static tool_id_t pending_register_tool = 0;
static on_macro_return_ptr on_changed_return   = NULL;

static void deferred_register_tool (void)
{
    grbl.on_macro_return = on_changed_return;
    on_changed_return = NULL;

    if(pending_register_tool > 0) {
        tool_id_t tool_id = pending_register_tool;
        pending_register_tool = 0;
        tooltable_register_tool(tool_id, NULL);
    }

    if(grbl.on_macro_return)
        grbl.on_macro_return();
}

static void onToolChanged (tool_data_t *tool)
{
    if(fs_available && tool->tool_id > 0) {
        if(hal.stream.file != NULL) {
            pending_register_tool = tool->tool_id;
            on_changed_return       = grbl.on_macro_return;
            grbl.on_macro_return  = deferred_register_tool;
        } else {
            tooltable_register_tool(tool->tool_id, NULL);
        }
    }

    current_tool = tool->tool_id;

    if(on_tool_changed)
        on_tool_changed(tool);
}

static void onToolSelect (tool_data_t *tool, bool next)
{
    //char buf[128];
    //sprintf(buf, "[onToolSelect: tool_id=%u next=%d current_tool=%u gc_state.tool=%u]" ASCII_EOL,
    //tool->tool_id, next, current_tool, gc_state.tool ? gc_state.tool->tool_id : 0);
    //hal.stream.write(buf);
 
    if(!next) {
        current_tool = tool->tool_id;
        ngc_param_set(4904, 1.0f);      // M61 signal: not a real Txx
    }
 
    if(next && max_pockets > 0) {
        // Use grblHAL's current tool data directly — reliable on boot
        // since grblHAL restores it from persistent storage before we run
        tool_id_t outgoing_id = gc_state.tool ? gc_state.tool->tool_id : 0;
 
        // Look up outgoing pocket directly from file — permanent assignment
        tool_pocket_t outgoing;
        pocket_id_t outgoing_pocket = -1;
        if(outgoing_id > 0 && file_find(outgoing_id, &outgoing) && outgoing.pocket_id >= 1)
            outgoing_pocket = outgoing.pocket_id;
 
        // Look up incoming pocket directly from file
        tool_pocket_t incoming;
        pocket_id_t incoming_pocket = -1;
        if(file_find(tool->tool_id, &incoming) && incoming.pocket_id >= 1)
            incoming_pocket = incoming.pocket_id;
 
        // Report incoming tool ID and name to operator
        char buf[80];
        if(incoming.name[0] != '\0')
            snprintf(buf, sizeof(buf), "T%ld: %s" ASCII_EOL, (long)tool->tool_id, incoming.name);
        else
            snprintf(buf, sizeof(buf), "T%ld" ASCII_EOL, (long)tool->tool_id);
        report_message(buf, Message_Info);
 
        ngc_param_set(4900, (float)outgoing_id);
        ngc_param_set(4901, (float)outgoing_pocket);
        ngc_param_set(4902, (float)tool->tool_id);
        ngc_param_set(4903, (float)incoming_pocket);
        ngc_param_set(4904, 0.0f);      // signal: real Txx pre-selection
    }
 
    if(tool_select)
        tool_select(tool, next);
}

// $TTLIST - print tool table to console directly from file.
// ---------------------------------------------------------------------------
static status_code_t list_tools (sys_state_t state, char *args)
{
    hal.stream.write("[TOOLTABLE: P=pocket T=tool offsets diameter name]" ASCII_EOL);

    if(!fs_available) {
        hal.stream.write("[TOOL: table is empty]" ASCII_EOL);
        return Status_OK;
    }

    vfs_file_t *file = vfs_open(filename, "r");
    if(!file) {
        hal.stream.write("[TOOL: file not accessible]" ASCII_EOL);
        return Status_FileReadError;
    }

    char line[300], buf[200], tmp[24];
    tool_pocket_t entry;
    bool any = false;

    while(read_line(file, line, sizeof(line))) {

        if(!parse_line(line, &entry))
            continue;

        any = true;

        uint16_t file_pocket = (entry.pocket_id >= 1) ? (uint16_t)entry.pocket_id : 0;
        sprintf(buf, "[TOOL: P%u T%u", file_pocket, (uint16_t)entry.tool.tool_id);

        for(uint_fast8_t axis = 0; axis < N_AXIS; axis++) {
            sprintf(tmp, " %s%.3f", axis_letter[axis], entry.tool.offset.values[axis]);
            strcat(buf, tmp);
        }

        if(entry.tool.radius != 0.0f) {
            sprintf(tmp, " D%.3f", entry.tool.radius * 2.0f);
            strcat(buf, tmp);
        }

        if(*entry.name) {
            strcat(buf, " ;");
            strcat(buf, entry.name);
        }

        strcat(buf, "]" ASCII_EOL);
        hal.stream.write(buf);
    }

    vfs_close(file);

    if(!any)
        hal.stream.write("[TOOL: table is empty]" ASCII_EOL);

    return Status_OK;
}

// ---------------------------------------------------------------------------
// $TTLOAD - verify tooltable file is accessible
// ---------------------------------------------------------------------------
static status_code_t load_tools (sys_state_t state, char *args)
{
    if(!fs_available)
        return Status_FileReadError;

    vfs_file_t *file = vfs_open(filename, "r");
    if(!file)
        return Status_FileReadError;

    vfs_close(file);
    report_message("Tooltable: reloaded", Message_Info);
    return Status_OK;
}

static status_code_t reload_tools (void)
{
    return load_tools(state_get(), NULL);
}

// ---------------------------------------------------------------------------
// File management
// ---------------------------------------------------------------------------
static void ensure_tooltable_exists (void)
{
    vfs_file_t *file = vfs_open(filename, "r");
    if(file) {
        vfs_close(file);
        return;
    }

    file = vfs_open(filename, "w");
    if(file) {
        vfs_close(file);
        report_message("Tooltable: created /tooltable.tbl", Message_Info);
    } else {
        report_message("Tooltable: failed to create /tooltable.tbl", Message_Warning);
    }
}

static void loadTools (const char *path, const vfs_t *fs, vfs_st_mode_t mode)
{
    fs_available = true;
    ensure_tooltable_exists();

    if(on_vfs_mount)
        on_vfs_mount(path, fs, mode);
}

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------
static void onReportOptions (bool newopt)
{
    on_report_options(newopt);
    if(!newopt)
        report_plugin("Tool table", "0.04");
}

// $TTREG=Tn [,name] — Register a tool in the tooltable at P0.
// ---------------------------------------------------------------------------
static status_code_t register_tool (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TTREG: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if(!args || !*args || (*args != 'T' && *args != 't')) {
        report_message("TTREG: usage is $TTREG=Tn [,name]", Message_Warning);
        return Status_BadNumberFormat;
    }

    uint32_t tool_id;
    const char *name = NULL;

    uint_fast8_t cc = 1;
    status_code_t parse_status = read_uint(args, &cc, &tool_id);
    if(parse_status != Status_OK) {
        report_message("TTREG: invalid tool number", Message_Warning);
        return parse_status;
    }
    while(args[cc] == ' ' || args[cc] == '\t') cc++;
    if(args[cc] == ',')
        name = &args[cc + 1];

    carousel_op_result_t result = tooltable_register_tool((tool_id_t)tool_id, name);

    switch(result) {
        case CarouselOp_OK:
            {
                char msg[80];
                if(name && *name)
                    snprintf(msg, sizeof(msg), "Tool %lu updated in tooltable (%s)", (unsigned long)tool_id, name);
                else
                    snprintf(msg, sizeof(msg), "Tool %lu registered in tooltable", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_AlreadyRegistered:
            {
                char msg[60];
                snprintf(msg, sizeof(msg), "Tool %lu already in tooltable — no changes made", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_TableNotLoaded:
            report_message("TTREG: tool table not loaded", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_WriteError:
            report_message("TTREG: failed to write tool table", Message_Warning);
            return Status_FileReadError;

        default:
            report_message("TTREG: unknown error", Message_Warning);
            return Status_GcodeValueOutOfRange;
    }
}

// $TTDEL=Tn — Delete a P0 tool entry from the tooltable entirely.
// ---------------------------------------------------------------------------
static status_code_t delete_tool (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TTDEL: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    if(!args || !*args || (*args != 'T' && *args != 't')) {
        report_message("TTDEL: usage is $TTDEL=Tn", Message_Warning);
        return Status_BadNumberFormat;
    }

    uint32_t tool_id;
    uint_fast8_t cc = 1;
    status_code_t parse_status = read_uint(args, &cc, &tool_id);
    if(parse_status != Status_OK) {
        report_message("TTDEL: invalid tool number", Message_Warning);
        return parse_status;
    }

    carousel_op_result_t result = tooltable_delete((tool_id_t)tool_id);

    switch(result) {
        case CarouselOp_OK:
            {
                char msg[48];
                sprintf(msg, "Tool %lu deleted from tooltable", (unsigned long)tool_id);
                report_message(msg, Message_Info);
            }
            return Status_OK;

        case CarouselOp_ToolNotFound:
            report_message("TTDEL: tool not found in tooltable", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_ToolAlreadyInPocket:
            report_message("TTDEL: tool is in a carousel pocket — use $TCRM first", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_TableNotLoaded:
            report_message("TTDEL: tool table not loaded", Message_Warning);
            return Status_GcodeValueOutOfRange;

        case CarouselOp_WriteError:
            report_message("TTDEL: failed to write tool table", Message_Warning);
            return Status_FileReadError;

        default:
            report_message("TTDEL: unknown error", Message_Warning);
            return Status_GcodeValueOutOfRange;
    }
}

// $TTCLR [Tn] — Clear the length offset for a tool in the tooltable.
// If no tool number is given, clears the offset for the current spindle tool.
// ---------------------------------------------------------------------------
static status_code_t clear_tool_offset (sys_state_t state, char *args)
{
    if(state_get() != STATE_IDLE) {
        report_message("TTCLR: machine must be IDLE", Message_Warning);
        return Status_InvalidStatement;
    }

    uint32_t tool_id;

    if(!args || !*args) {
        if(!gc_state.tool || gc_state.tool->tool_id == 0) {
            report_message("TTCLR: no tool in spindle and no tool number given", Message_Warning);
            return Status_BadNumberFormat;
        }
        tool_id = (uint32_t)gc_state.tool->tool_id;
    } else {
        if(*args != 'T' && *args != 't') {
            report_message("TTCLR: usage is $TTCLR [Tn]", Message_Warning);
            return Status_BadNumberFormat;
        }
        uint_fast8_t cc = 1;
        status_code_t parse_status = read_uint(args, &cc, &tool_id);
        if(parse_status != Status_OK) {
            report_message("TTCLR: invalid tool number", Message_Warning);
            return parse_status;
        }
    }

    tool_pocket_t existing;
    if(!file_find((tool_id_t)tool_id, &existing)) {
        report_message("TTCLR: tool not found in tooltable", Message_Warning);
        return Status_GcodeValueOutOfRange;
    }

    memset(&existing.tool.offset, 0, sizeof(existing.tool.offset));

    if(!grbl.tool_table.set_tool(&existing.tool)) {
        report_message("TTCLR: failed to write tool table", Message_Warning);
        return Status_FileReadError;
    }

    char msg[48];
    snprintf(msg, sizeof(msg), "Tool %lu offset cleared", (unsigned long)tool_id);
    report_message(msg, Message_Info);

    return Status_OK;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void tooltable_init (void)
{
    static const sys_command_t tt_command_list[] = {
        { "TTLOAD",  load_tools,        {}, { .str = "(re)load tool table from SD card" } },
        { "TTLIST",  list_tools,        {}, { .str = "List all tools in the tool table" } },
        { "TTREG",   register_tool,     {}, { .str = "Register a tool at P0 in the tooltable: $TTREG=Tn[,name]" } },
        { "TTDEL",   delete_tool,       {}, { .str = "Delete a P0 tool entry from the tooltable: $TTDEL=Tn" } },
        { "TTCLR",   clear_tool_offset, {}, { .str = "Clear length offset for a tool: $TTCLR [Tn]" } }
    };

    static sys_commands_t tt_commands = {
        .n_commands = sizeof(tt_command_list) / sizeof(sys_command_t),
        .commands   = tt_command_list
    };

    on_vfs_mount = vfs.on_mount;
    vfs.on_mount = loadTools;

    tool_select = hal.tool.select;
    hal.tool.select = onToolSelect;

    on_tool_changed = grbl.on_tool_changed;
    grbl.on_tool_changed = onToolChanged;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    // Initialise pocket0 as a safe zeroed fallback — tool_id=0, all offsets=0.
    // This mirrors the TOOLTABLE_ENABLE==1 pattern so getTool/getToolByIdx
    // always return valid data even before the VFS mounts or if the table is empty.
    memset(&pocket0, 0, sizeof(tool_pocket_t));
    pocket0.tool.tool_id = 0;
    pocket0.pocket_id    = -1;
    
    grbl.tool_table.n_tools         = 1;
    grbl.tool_table.get_tool        = getTool;
    grbl.tool_table.reload          = reload_tools;
    grbl.tool_table.set_tool        = setTool;
    grbl.tool_table.get_tool_by_idx = getToolByIdx;
    grbl.tool_table.clear           = clearTools;

    system_register_commands(&tt_commands);

    settings.macro_atc_flags.random_toolchanger = 1;

    // Seed current_tool from grblHAL's persisted spindle tool on boot
    current_tool = gc_state.tool ? gc_state.tool->tool_id : 0;

#if SDCARD_ENABLE
    sdcard_early_mount();
#endif
}

#endif // TOOLTABLE_ENABLE