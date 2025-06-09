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
void motion_enqueue_home(int axis);
motion_state_t motion_get_current_state();
motion_stop_reason_t motion_get_last_stop_reason();

/** Set how many microsteps are needed for moving the corresponding axis in
 * +1unit (+1 mm or +1 rotation).
 *
 * TODO: Calling this after movement results in weird behavior.
 */
void motion_set_motor_unitsteps(int motor_num, float unitsteps);

/** Called by settings system when home settings change */
void motion_set_home_origin(int axis, float origin_mm);
void motion_set_home_side(int axis, float side);
