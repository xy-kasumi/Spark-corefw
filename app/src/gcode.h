// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * G-code execution (main entry point for G/M-code commands).
 */
#pragma once

#include "coords.h"

/**
 * Execute a G-code or M-code command.
 * @param full_command The complete G/M-code command string
 */
void exec_gcode(char* full_command);

/**
 * Get current coordinate system.
 * @return Current coordinate system mode
 */
coord_system_t gcode_get_current_coord_system();

/**
 * Set a single coordinate system offset value (called by settings system).
 * @param cs_type Coordinate system (COORD_SYSTEM_GRINDER or COORD_SYSTEM_WORK)
 * @param axis Axis to set
 * @param value Offset value in machine coordinates
 */
void gcode_set_coord_offset(coord_system_t cs_type, axis_t axis, float value);

/**
 * Get current coordinate system offsets.
 * @return Current coordinate system offsets
 */
const coord_offsets_t* gcode_get_coord_offsets();
