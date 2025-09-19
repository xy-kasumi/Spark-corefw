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
 * Represents why motion stopped.
 */
typedef enum {
  STOP_REASON_TARGET_REACHED,
  STOP_REASON_PROBE_TRIGGERED,
  STOP_REASON_STALL_DETECTED,
  STOP_REASON_CANCELLED,  // Stopped due to cancel request
} motion_stop_reason_t;

/**
 * (blocking) Initialize motion subsystem.
 */
bool motion_init();

/**
 * Start an EDM move. Can accept further motion_move_enqueue_pos().
 */
void motion_start_edm_move(pos_phys_t to_pos);

/**
 * Start fast move. Can accept further motion_move_enqueue_pos().
 */
void motion_start_fast_move(pos_phys_t to_pos);

/**
 * Start probe move. Can accept further motion_move_enqueue_pos().
 */
void motion_start_probe_move(pos_phys_t to_pos);

/**
 * Home for the specified axis. Must not call motion_move_enqueue_pos().
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
void motion_move_enqueue_pos(pos_phys_t to_pos);

/**
 * Returns current position in machine coordinates.
 * TODO: must be thread safe for use in signal handler.
 */
pos_phys_t motion_get_current_pos();

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
