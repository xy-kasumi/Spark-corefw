// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "motion.h"

#include "canceler.h"
#include "comm.h"
#include "coords.h"
#include "motor.h"
#include "pulser.h"

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
static float home_travels[3] = {500.0f, 500.0f, 500.0f};

// Homing offset: bridges gap between driver coords and physical coords
// Updated after each successful home operation (X, Y, Z only - C has no home)
static pos_drv_t homing_offset = {0, 0, 0, 0};

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
// Convert axis to motor number (-1 if not found)
static int axis_to_motor(axis_t axis) {
  switch (axis) {
    case AXIS_X:
      return MOTOR_X;
    case AXIS_Y:
      return MOTOR_Y;
    case AXIS_Z:
      return MOTOR_Z;
    case AXIS_C:
      return MOTOR_C;
    default:
      return -1;
  }
}

static void update_homing_offset(axis_t axis, pos_phys_t* current_pos) {
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
  switch (axis) {
    case AXIS_X:
      homing_offset.x = current_drv.x - raw_expected.x;
      break;
    case AXIS_Y:
      homing_offset.y = current_drv.y - raw_expected.y;
      break;
    case AXIS_Z:
      homing_offset.z = current_drv.z - raw_expected.z;
      break;
    default:
      break;
  }

  // Set physical position to homing origin
  switch (axis) {
    case AXIS_X:
      current_pos->x = home_origins[AXIS_X];
      break;
    case AXIS_Y:
      current_pos->y = home_origins[AXIS_Y];
      break;
    case AXIS_Z:
      current_pos->z = home_origins[AXIS_Z];
      break;
    default:
      break;
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
static bool homing;
static axis_t homing_axis;  // Valid only when homing

// Timer for periodic tick
static struct k_timer motion_timer;

// only called from motion_tick_handler
static void stop_motion(motion_stop_reason_t reason) {
  if (homing) {
    update_homing_offset(homing_axis, &pos);
  }
  last_stop_reason = reason;
  state = MOTION_STATE_STOPPED;
}

static void motion_tick_handler(struct k_timer* timer) {
  if (state != MOTION_STATE_MOVING) {
    return;
  }

  // Check for cancellation first (highest priority)
  if (canceler_cancel_needed()) {
    last_stop_reason = STOP_REASON_CANCELLED;
    state = MOTION_STATE_STOPPED;
    return;
  }

  // Check for stall condition
  if (stop_at_stall) {
    int motor_num = axis_to_motor(homing_axis);
    if (motor_num >= 0 && motor_stalled(motor_num)) {
      stop_motion(STOP_REASON_STALL_DETECTED);
      return;
    }
  }

  // Check for probe trigger
  if (stop_at_probe) {
    if (pulser_has_discharge()) {
      stop_motion(STOP_REASON_PROBE_TRIGGERED);
      return;
    }
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
    stop_motion(STOP_REASON_TARGET_REACHED);
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

bool motion_init() {
  // Initialize timer for 1ms periodic tick
  k_timer_init(&motion_timer, motion_tick_handler, NULL);
  k_timer_start(&motion_timer, K_MSEC(1), K_MSEC(1));

  comm_ps_k_vbool(PS_INIT, "motion.ok", true);
  return true;
}

pos_phys_t motion_get_current_pos() {
  return pos;
}

// Unified internal motion enqueue function
static void motion_enqueue_internal(pos_phys_t to_pos,
                                    movement_type_t move_type,
                                    float velocity,
                                    bool enable_stall_stop,
                                    bool enable_probe_stop,
                                    bool is_homing,
                                    axis_t home_axis) {
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
  pb_init(&motion_path, &pos, &to_pos);

  // Set movement type and velocity
  movement_type = move_type;
  movement_velocity = velocity;

  // Set stop conditions
  stop_at_stall = enable_stall_stop;
  stop_at_probe = enable_probe_stop;

  // Set homing state
  homing = is_homing;
  if (is_homing) {
    homing_axis = home_axis;
  }

  // Start moving
  state = MOTION_STATE_MOVING;
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

void motion_set_home_travel(axis_t axis, float travel_mm) {
  if (axis == AXIS_X || axis == AXIS_Y || axis == AXIS_Z) {
    home_travels[axis] = travel_mm;
  }
}

void motion_start_fast_move(pos_phys_t to_pos) {
  motion_enqueue_internal(to_pos, MOVEMENT_CONSTANT_VELOCITY, VELOCITY_MM_PER_S,
                          false, false, false, AXIS_X);
}

void motion_start_edm_move(pos_phys_t to_pos) {
  edm_current_speed = EDM_INITIAL_VELOCITY_MM_PER_S;
  motion_enqueue_internal(to_pos, MOVEMENT_EDM_CONTROL, 0.0f, false, false,
                          false, AXIS_X);
}

void motion_start_probe_move(pos_phys_t to_pos) {
  motion_enqueue_internal(to_pos, MOVEMENT_CONSTANT_VELOCITY,
                          PROBE_VELOCITY_MM_PER_S, false, true, false, AXIS_X);
}

void motion_start_home(axis_t axis) {
  // Validate axis (X, Y, Z only - C has no home)
  if (axis != AXIS_X && axis != AXIS_Y && axis != AXIS_Z) {
    return;
  }

  // Calculate target position for homing
  pos_phys_t home_target = pos;
  float side = home_sides[axis];
  float travel = home_travels[axis];
  if (axis == AXIS_X) {
    home_target.x += side * travel;
  } else if (axis == AXIS_Y) {
    home_target.y += side * travel;
  } else if (axis == AXIS_Z) {
    home_target.z += side * travel;
  }

  // Execute
  motion_enqueue_internal(home_target, MOVEMENT_CONSTANT_VELOCITY,
                          VELOCITY_MM_PER_S, true, false, true, axis);
}

bool motion_move_can_enqueue() {
  return pb_can_write(&motion_path);
}

void motion_move_enqueue_pos(pos_phys_t to_pos) {
  if (!pb_can_write(&motion_path)) {
    // should not happen
    return;
  }
  pb_write(&motion_path, &to_pos);
}

bool motion_is_stopped(motion_stop_reason_t* reason) {
  if (state == MOTION_STATE_STOPPED) {
    if (reason) {
      *reason = last_stop_reason;
    }
    return true;
  } else {
    return false;
  }
}
