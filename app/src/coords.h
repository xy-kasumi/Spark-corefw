// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Stateless) Coordinate system transformation utilities and fundamental
 * position type.
 */
#pragma once

/** Represents a single axis */
typedef enum {
  AXIS_NONE = -1,  // No axis / invalid axis
  AXIS_X = 0,
  AXIS_Y = 1,
  AXIS_Z = 2,
  AXIS_C = 3,
} axis_t;

/** Represents a single physical coordinate. (i.e. coordinates specification in
 * G-code)
 */
typedef struct {
  float x;
  float y;
  float z;
  float c;  // turns in [0, 1)
} pos_phys_t;

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

/** Compute distance between two pos_phys_t points.
 *
 * TODO: definition of "distance" is very weird.
 * @return distance in mm
 */
float posp_dist(const pos_phys_t* a, const pos_phys_t* b);

/** Linearly interpolate between a (t=0), and b (t=1).
 * t can be outside of [0, 1] range, in which case it will be extrapolated.
 */
void posp_interp(const pos_phys_t* a,
                 const pos_phys_t* b,
                 float t,
                 pos_phys_t* out);

/** Convert degrees to turns.
 * @param degrees Angle in degrees
 * @return Angle in turns
 */
float degrees_to_turns(float degrees);

/** Convert turns to degrees.
 * @param turns Angle in turns
 * @return Angle in degrees
 */
float turns_to_degrees(float turns);

/** Normalize C-axis value to [0, 1) range.
 * @param c C-axis value in turns
 * @return Normalized value in [0, 1) range
 */
float normalize_c_axis_turns(float c);

/** Calculate shortest path rotation from current to target C-axis position.
 * @param current Current C-axis position in [0, 1)
 * @param target Target C-axis position in [0, 1)
 * @return Shortest rotation delta (-0.5, 0.5]
 */
float c_axis_shortest_path_turns(float current, float target);
