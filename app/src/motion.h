// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Motion controller & coordinate management.
 */
#pragma once

#include "motion_base.h"

/**
 * Represents motion state.
 */
typedef enum {
  MOTION_STATE_STOPPED,
  MOTION_STATE_MOVING,
} motion_state_t;

/**
 * Represents why motion stops (bitmask).
 * Can be OR-ed together to specify multiple stop conditions.
 * When STOP_REASON_NONE, only cancel can stop the motion.
 */
typedef enum {
  STOP_REASON_NONE = 0,           // No stop conditions
  STOP_REASON_TARGET = (1 << 0),  // Stop at target position
  STOP_REASON_PROBE = (1 << 1),   // Stop when probe triggers
  STOP_REASON_STALL = (1 << 2),   // Stop when motor stalls
} motion_stop_reason_t;

/**
 * Data for representing "edm" p-state.
 */
typedef struct {
  bool has_edm_data;
  float r_open;
  float r_short;
  float temp;

  bool is_moving;
  float pb_front;
  float pb_back;
  float distance;
  float distance_max;
} ps_edm_t;

/**
 * (blocking) Initialize motion subsystem.
 */
bool motion_init();

/**
 * (blocking) Handle stat command.
 */
void motion_dump_status();

/**
 * Start an EDM move. Only allowed when not moving.
 * Can accept further motion_move_enqueue_pos().
 */
void motion_start_edm_move(pos_phys_t to_pos, bool has_cont);

/**
 * Start fast move. Only allowed when not moving.
 * Can accept further motion_move_enqueue_pos().
 */
void motion_start_fast_move(pos_phys_t to_pos);

/**
 * Start probe move. Only allowed when not moving.
 * Can accept further motion_move_enqueue_pos().
 */
void motion_start_probe_move(pos_phys_t to_pos);

/**
 * Home for the specified axis. Only allowed when not moving.
 * Must not call motion_move_enqueue_pos().
 * @param axis Axis to home (X, Y, or Z only - C axis has no home)
 */
void motion_start_home(axis_t axis);

/**
 * Returns true if motion is stopped.
 * @param reason (optional) store reason when true is returned.
 */
bool motion_is_stopped(motion_stop_reason_t* reason);

/**
 * Returns true if motion can accept motion_queue_pos() calls.
 */
bool motion_move_can_enqueue();

/**
 * Tell next point to move to.
 */
void motion_move_enqueue_pos(pos_phys_t to_pos, bool has_cont);

/**
 * Returns current position in machine coordinates. (thread-safe)
 */
pos_phys_t motion_get_current_pos();

/**
 * Returns latest edm state. (thread-safe)
 */
ps_edm_t motion_get_edm_state();

/** Set how many microsteps are needed for moving the corresponding axis in
 * +1unit (+1 mm or +1 rotation).
 *
 * TODO: Calling this after movement results in weird behavior.
 */
void motion_set_motor_unitsteps(int motor_num, float unitsteps);

/**
 * Set home origin position for an axis (called by settings system).
 * @param axis Axis to configure (X, Y, or Z only)
 * @param origin_mm Origin position in mm
 */
void motion_set_home_origin(axis_t axis, float origin_mm);

/**
 * Set home side/direction for an axis (called by settings system).
 * @param axis Axis to configure (X, Y, or Z only)
 * @param side Direction to move for homing (+1 or -1)
 */
void motion_set_home_side(axis_t axis, float side);

/**
 * Set home travel distance for an axis (called by settings system).
 * @param axis Axis to configure (X, Y, or Z only)
 * @param travel_mm Maximum distance (mm) to travel when homing
 */
void motion_set_home_travel(axis_t axis, float travel_mm);
