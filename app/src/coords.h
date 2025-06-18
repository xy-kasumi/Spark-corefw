// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Stateless) Coordinate system transformation utilities.
 */
#pragma once

#include "motion_base.h"

/** Supported coordinate systems */
typedef enum {
  COORD_SYSTEM_MACHINE,  // G53 - Machine coordinates (default)
  COORD_SYSTEM_GRINDER,  // G54 - Grinder coordinates
  COORD_SYSTEM_WORK,     // G55 - Work coordinates
} coord_system_t;

/** Coordinate system offsets.
 * All origins are stored in machine coordinates.
 */
typedef struct {
  pos_phys_t grinder_origin;  // G54 origin in machine coords
  pos_phys_t work_origin;     // G55 origin in machine coords
} coord_offsets_t;

/** Convert position from specified coordinate system to machine coordinates.
 * @param pos Position in source coordinate system
 * @param from_cs Source coordinate system
 * @param offsets Current coordinate system offsets
 * @return Position in machine coordinates
 */
pos_phys_t coords_to_machine(const pos_phys_t* pos,
                              coord_system_t from_cs,
                              const coord_offsets_t* offsets);

/** Convert position from machine coordinates to specified coordinate system.
 * @param machine_pos Position in machine coordinates
 * @param to_cs Target coordinate system
 * @param offsets Current coordinate system offsets
 * @return Position in target coordinate system
 */
pos_phys_t coords_from_machine(const pos_phys_t* machine_pos,
                                coord_system_t to_cs,
                                const coord_offsets_t* offsets);