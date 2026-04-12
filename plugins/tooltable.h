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
