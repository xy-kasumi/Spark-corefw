// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "wirefeed.h"

#include "comm.h"
#include "motor.h"
#include "system.h"

#include <zephyr/kernel.h>

// Configuration
static float motor_unitsteps = 200.0f;      // Steps per mm for motor6
static const float TICK_PERIOD_S = 0.001f;  // 1ms tick period

// State
typedef enum {
  WIREFEED_STATE_STOPPED,
  WIREFEED_STATE_FEEDING
} wirefeed_state_t;

static wirefeed_state_t state = WIREFEED_STATE_STOPPED;
static float current_pos_mm = 0.0f;
static float feedrate_mm_per_min = 0.0f;
static float mm_per_tick = 0.0f;  // Calculated from feedrate

// Timer for periodic tick
static struct k_timer wirefeed_timer;

static void wirefeed_tick_handler(struct k_timer* timer) {
  if (state != WIREFEED_STATE_FEEDING) {
    return;
  }

  // Check for cancellation
  if (g_cancel_requested) {
    state = WIREFEED_STATE_STOPPED;
    return;
  }

  // Update position
  current_pos_mm += mm_per_tick;

  // Calculate target in microsteps
  int target_steps = (int)(current_pos_mm * motor_unitsteps);

  // Update motor6 target position
  motor_set_target_steps(6, target_steps);
}

void wirefeed_init() {
  comm_print("wirefeed: init");

  // Initialize timer
  k_timer_init(&wirefeed_timer, wirefeed_tick_handler, NULL);
  k_timer_start(&wirefeed_timer, K_MSEC(1), K_MSEC(1));  // 1ms period

  comm_print("wirefeed: ready");
}

void wirefeed_start(float feedrate_mm_per_min_arg) {
  feedrate_mm_per_min = feedrate_mm_per_min_arg;
  mm_per_tick = (feedrate_mm_per_min / 60.0f) * TICK_PERIOD_S;
  state = WIREFEED_STATE_FEEDING;
  comm_print("wirefeed: start feed=%.3f mm/min", (double)feedrate_mm_per_min);
}

void wirefeed_stop() {
  state = WIREFEED_STATE_STOPPED;
  comm_print("wirefeed: stop");
}

void wirefeed_set_unitsteps(float unitsteps) {
  motor_unitsteps = unitsteps;
  comm_print("wirefeed: unitsteps=%.3f", (double)unitsteps);
}

void wirefeed_dump_status() {
  comm_print("=== Wirefeed Status ===");
  comm_print("  state: %s",
             state == WIREFEED_STATE_FEEDING ? "FEEDING" : "STOPPED");
  comm_print("  position: %.3f mm", (double)current_pos_mm);
  comm_print("  feedrate: %.3f mm/min", (double)feedrate_mm_per_min);
  comm_print("  unitsteps: %.3f steps/mm", (double)motor_unitsteps);
}
