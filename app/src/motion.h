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
void motion_init();

pos_phys_t motion_get_current_pos();
void motion_enqueue_move(pos_phys_t to_pos);
void motion_enqueue_edm_move(pos_phys_t to_pos);
void motion_enqueue_probe(pos_phys_t to_pos);

/**
 * Enqueue a homing move for the specified axis.
 * @param axis Axis to home (X, Y, or Z only - C axis has no home)
 */
void motion_enqueue_home(axis_t axis);
motion_state_t motion_get_current_state();
motion_stop_reason_t motion_get_last_stop_reason();

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
