// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "motion.h"

#include "comm.h"
#include "coords.h"
#include "motor.h"
#include "pulser.h"
#include "system.h"

#include <drivers/tmc_driver.h>
#include <math.h>
#include <zephyr/kernel.h>

// Motor-axis mapping
#define MOTOR_X 0
#define MOTOR_Y 1
#define MOTOR_Z 2
#define MOTOR_C 5

// Motion constants
static const float VELOCITY_MM_PER_S = 10.0f;
static const float EDM_INITIAL_VELOCITY_MM_PER_S = 0.5f;  // Start slow for EDM
static const float PROBE_VELOCITY_MM_PER_S = 1.0f;
static const float TICK_PERIOD_S = 0.001f;  // 1ms tick period in seconds

// Local position type for motion-controlled axes only
typedef struct {
  int x, y, z, c;  // microsteps. (for infinite rotary axes, it's modulo)
} pos_drv_t;

// Motor configuration (pushed from settings)
static float motor_unitsteps[MOTOR_COUNT] = {200.0f, 200.0f, 200.0f, 200.0f,
                                             200.0f, 200.0f, 200.0f};

// Home configuration (pushed from settings) - X, Y, Z only
static float home_origins[3] = {0.0f, 0.0f, 0.0f};
static float home_sides[3] = {1.0f, 1.0f, 1.0f};

// Homing offset: bridges gap between driver coords and physical coords
// Updated after each successful home operation (X, Y, Z only - C has no home)
static pos_drv_t homing_offset = {0, 0, 0, 0};

// Constants
static const float MAX_TRAVEL_MM = 500.0f;

// Convert physical position to driver coordinates (microsteps)
static pos_drv_t phys_to_drv(pos_phys_t phys) {
  // Convert to raw driver steps
  pos_drv_t raw_drv = {.x = (int)(phys.x * motor_unitsteps[MOTOR_X]),
                       .y = (int)(phys.y * motor_unitsteps[MOTOR_Y]),
                       .z = (int)(phys.z * motor_unitsteps[MOTOR_Z]),
                       .c = (int)(phys.c * motor_unitsteps[MOTOR_C])};

  // Apply homing offset to linear axes only (C has no home)
  return (pos_drv_t){.x = raw_drv.x + homing_offset.x,
                     .y = raw_drv.y + homing_offset.y,
                     .z = raw_drv.z + homing_offset.z,
                     .c = raw_drv.c};  // No offset for C-axis
}

// Update homing offset after successful homing (X, Y, Z only)
static void update_homing_offset(axis_t axis) {
  // Get current driver position (where we actually are)
  pos_drv_t current_drv = {.x = motor_get_current_steps(MOTOR_X),
                           .y = motor_get_current_steps(MOTOR_Y),
                           .z = motor_get_current_steps(MOTOR_Z)};

  // Calculate where driver coordinates should be for the new physical origin
  pos_phys_t origin_phys = {home_origins[0], home_origins[1], home_origins[2],
                            0.0f};
  pos_drv_t raw_expected = {
      .x = (int)(origin_phys.x * motor_unitsteps[MOTOR_X]),
      .y = (int)(origin_phys.y * motor_unitsteps[MOTOR_Y]),
      .z = (int)(origin_phys.z * motor_unitsteps[MOTOR_Z])};

  // Update offset for the homed axis so current driver position maps to origin
  if (axis == AXIS_X) {
    homing_offset.x = current_drv.x - raw_expected.x;
  } else if (axis == AXIS_Y) {
    homing_offset.y = current_drv.y - raw_expected.y;
  } else if (axis == AXIS_Z) {
    homing_offset.z = current_drv.z - raw_expected.z;
  }
}

// Motion state
static pos_phys_t pos;
static motion_state_t state = MOTION_STATE_STOPPED;

// Motion planning state
static path_buffer_t motion_path;

// Movement type
typedef enum {
  MOVEMENT_CONSTANT_VELOCITY,
  MOVEMENT_EDM_CONTROL,
} movement_type_t;

// Movement control state (orthogonal to stop conditions)
static movement_type_t movement_type;
static float movement_velocity;         // For MOVEMENT_CONSTANT_VELOCITY
static float edm_current_speed = 0.0f;  // For MOVEMENT_EDM_CONTROL

// Stop condition flags
static motion_stop_reason_t last_stop_reason;
static bool stop_at_probe;
static bool stop_at_stall;
static axis_t homing_axis;  // Valid only when stop_at_stall is true

// Timer for periodic tick
static struct k_timer motion_timer;

static void motion_tick_handler(struct k_timer* timer) {
  if (state != MOTION_STATE_MOVING) {
    return;
  }

  // Check for cancellation first (highest priority)
  if (g_cancel_requested) {
    last_stop_reason = STOP_REASON_CANCELLED;
    state = MOTION_STATE_STOPPED;
    return;
  }

  // Check for stall condition (homing)
  if (stop_at_stall) {
    const struct device* motor = motor_get_device(homing_axis);
    if (motor && tmc_stalled(motor)) {
      // Homing completed - stall detected
      // Update homing offset before changing physical position
      update_homing_offset(homing_axis);

      // Set physical position to homing origin
      pos.x = (homing_axis == AXIS_X) ? home_origins[AXIS_X] : pos.x;
      pos.y = (homing_axis == AXIS_Y) ? home_origins[AXIS_Y] : pos.y;
      pos.z = (homing_axis == AXIS_Z) ? home_origins[AXIS_Z] : pos.z;

      last_stop_reason = STOP_REASON_STALL_DETECTED;
      state = MOTION_STATE_STOPPED;
      return;
    }
  }

  // Check for probe trigger (if enabled)
  if (stop_at_probe) {
    // TODO: Add actual probe detection logic when probe hardware is available
    // For now, probe moves will only stop at path end
  }

  // Movement control
  if (movement_type == MOVEMENT_EDM_CONTROL) {
    // EDM control logic
    uint8_t open_rate = pulser_get_open_rate();
    uint8_t short_rate = pulser_get_short_rate();

    if (open_rate > 127) {
      // too much open: too far away
      pb_move(&motion_path, 1e-3f);  // +1 um / tick (-> 1mm/s max)
    } else if (short_rate > 127) {
      // too much short: too close
      pb_move(&motion_path, -5e-3f);  // -5 um / tick (-> -5mm/s max)
    }
  } else {
    // Constant velocity movement
    pb_move(&motion_path, movement_velocity * TICK_PERIOD_S);
  }
  pos = pb_get_pos(&motion_path);

  // Check if path completed
  if (pb_at_end(&motion_path)) {
    last_stop_reason = STOP_REASON_TARGET_REACHED;
    state = MOTION_STATE_STOPPED;
    return;
  }

  // Convert to driver coordinates and send to motors
  pos_drv_t target_drv = phys_to_drv(pos);
  motor_set_target_steps(MOTOR_X, target_drv.x);
  motor_set_target_steps(MOTOR_Y, target_drv.y);
  motor_set_target_steps(MOTOR_Z, target_drv.z);

  // C-axis uses modulo logic for shortest path
  int c_modulo_steps =
      (int)motor_unitsteps[MOTOR_C];  // 1 turn = motor_unitsteps[MOTOR_C] steps
  motor_set_target_steps_with_modulo(MOTOR_C, target_drv.c, c_modulo_steps);
}

void motion_init() {
  // Initialize timer for 1ms periodic tick
  k_timer_init(&motion_timer, motion_tick_handler, NULL);
  k_timer_start(&motion_timer, K_MSEC(1), K_MSEC(1));

  comm_print("motion: init ok (1ms tick)");
}

pos_phys_t motion_get_current_pos() {
  return pos;
}

// Unified internal motion enqueue function
static void motion_enqueue_internal(pos_phys_t to_pos,
                                    movement_type_t move_type,
                                    float velocity,
                                    bool enable_stall_stop,
                                    bool enable_probe_stop) {
  // Don't start new move if already moving
  if (state == MOTION_STATE_MOVING) {
    return;
  }

  // Skip if no movement needed
  float distance = posp_dist(&pos, &to_pos);
  if (distance < 0.001f) {
    return;
  }

  // Initialize path buffer with single segment
  pb_init(&motion_path, &pos, &to_pos, true);  // Single segment, end=true

  // Set movement type and velocity
  movement_type = move_type;
  movement_velocity = velocity;

  // Set stop conditions
  stop_at_stall = enable_stall_stop;
  stop_at_probe = enable_probe_stop;

  // Start moving
  state = MOTION_STATE_MOVING;
}

motion_state_t motion_get_current_state() {
  return state;
}

void motion_set_motor_unitsteps(int motor_num, float unitsteps) {
  if (motor_num >= 0 && motor_num < MOTOR_COUNT) {
    motor_unitsteps[motor_num] = unitsteps;
  }
}

void motion_set_home_origin(axis_t axis, float origin_mm) {
  if (axis == AXIS_X || axis == AXIS_Y || axis == AXIS_Z) {
    home_origins[axis] = origin_mm;
  }
}

void motion_set_home_side(axis_t axis, float side) {
  if (axis == AXIS_X || axis == AXIS_Y || axis == AXIS_Z) {
    home_sides[axis] = side;
  }
}

void motion_enqueue_move(pos_phys_t to_pos) {
  motion_enqueue_internal(to_pos, MOVEMENT_CONSTANT_VELOCITY, VELOCITY_MM_PER_S,
                          false, false);
}

void motion_enqueue_edm_move(pos_phys_t to_pos) {
  edm_current_speed = EDM_INITIAL_VELOCITY_MM_PER_S;
  motion_enqueue_internal(to_pos, MOVEMENT_EDM_CONTROL, 0.0f, false, false);
}

void motion_enqueue_probe(pos_phys_t to_pos) {
  motion_enqueue_internal(to_pos, MOVEMENT_CONSTANT_VELOCITY,
                          PROBE_VELOCITY_MM_PER_S, false, true);
}

void motion_enqueue_home(axis_t axis) {
  // Validate axis (X, Y, Z only - C has no home)
  if (axis != AXIS_X && axis != AXIS_Y && axis != AXIS_Z) {
    return;
  }

  // Calculate target position for homing
  pos_phys_t home_target = pos;
  float side = home_sides[axis];
  if (axis == AXIS_X) {
    home_target.x += side * MAX_TRAVEL_MM;
  } else if (axis == AXIS_Y) {
    home_target.y += side * MAX_TRAVEL_MM;
  } else if (axis == AXIS_Z) {
    home_target.z += side * MAX_TRAVEL_MM;
  }

  // Set homing axis before starting motion
  homing_axis = axis;

  // Use internal function with stall detection enabled
  motion_enqueue_internal(home_target, MOVEMENT_CONSTANT_VELOCITY,
                          VELOCITY_MM_PER_S, true, false);
}

motion_stop_reason_t motion_get_last_stop_reason() {
  return last_stop_reason;
}
