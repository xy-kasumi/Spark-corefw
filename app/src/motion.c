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

/**
 * Data for motion stats.
 */
typedef struct {
  int start;
  int stop;
  int stop_probe;
  int stop_stall;
  int stop_target;
} motion_stat_cnt_t;

static void add_stop_cnt(motion_stat_cnt_t* cnt, motion_stop_reason_t reason) {
  cnt->stop++;
  switch (reason) {
    case STOP_REASON_TARGET:
      cnt->stop_target++;
      break;
    case STOP_REASON_PROBE:
      cnt->stop_probe++;
      break;
    case STOP_REASON_STALL:
      cnt->stop_stall++;
      break;
    default:
      break;
  }
}

// Latest EDM state (written only from ISR, read from various threads)
ps_edm_t latest_edm_state;
motion_stat_cnt_t motion_stats;

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
static motion_stop_reason_t last_stop_reason;

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

// Stop conditions & action
static motion_stop_reason_t stop_conditions;
static bool homing;
static axis_t homing_axis;  // Valid only when homing

// Periodic tick
static struct k_work_delayable motion_tick_work;

// only called from motion_tick_handler
static void stop_motion(motion_stop_reason_t reason) {
  if (homing) {
    update_homing_offset(homing_axis, &pos);
  }
  add_stop_cnt(&motion_stats, reason);
  last_stop_reason = reason;
  state = MOTION_STATE_STOPPED;
}

// (system workqueue) Periodic motion control
static void motion_tick_handler(struct k_work* work) {
  k_work_reschedule(&motion_tick_work, K_MSEC(1));

  latest_edm_state.has_edm_data = false;
  latest_edm_state.is_moving = false;

  if (state != MOTION_STATE_MOVING) {
    return;
  }

  // Check for cancellation first (highest priority)
  if (canceler_cancel_needed()) {
    stop_motion(STOP_REASON_NONE);
    return;
  }

  // Check specified stop conditions.
  if (stop_conditions & STOP_REASON_STALL) {
    int motor_num = axis_to_motor(homing_axis);
    if (motor_num >= 0 && motor_stalled(motor_num)) {
      stop_motion(STOP_REASON_STALL);
      return;
    }
  }

  if (stop_conditions & STOP_REASON_PROBE) {
    if (pulser_has_discharge()) {
      stop_motion(STOP_REASON_PROBE);
      return;
    }
  }

  if (stop_conditions & STOP_REASON_TARGET) {
    if (pb_at_end(&motion_path)) {
      stop_motion(STOP_REASON_TARGET);
      return;
    }
  }

  // Movement control
  if (movement_type == MOVEMENT_EDM_CONTROL) {
    // EDM control logic
    uint8_t open_rate = pulser_get_open_rate();
    uint8_t short_rate = pulser_get_short_rate();
    uint8_t temp = pulser_get_temp();

    latest_edm_state.has_edm_data = true;
    latest_edm_state.r_open = open_rate * (1 / 255.0f);
    latest_edm_state.r_short = short_rate * (1 / 255.0f);
    latest_edm_state.temp = temp;

    if (open_rate > 200) {
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

  // Update stats
  float dist = pb_get_distance(&motion_path);
  latest_edm_state.is_moving = true;
  latest_edm_state.pb_front = pb_get_forward_buffer(&motion_path);
  latest_edm_state.pb_back = pb_get_backward_buffer(&motion_path);
  latest_edm_state.distance = dist;
  latest_edm_state.distance_max = fmaxf(latest_edm_state.distance_max, dist);

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
  k_work_init_delayable(&motion_tick_work, motion_tick_handler);
  k_work_reschedule(&motion_tick_work, K_MSEC(1));

  comm_ps_k_vbool(PS_INIT, "motion.ok", true);
  return true;
}

pos_phys_t motion_get_current_pos() {
  pos_phys_t ret;
  unsigned int key = irq_lock();
  ret = pos;
  irq_unlock(key);
  return ret;
}

ps_edm_t motion_get_edm_state() {
  ps_edm_t ret;
  unsigned int key = irq_lock();
  ret = latest_edm_state;
  irq_unlock(key);
  return ret;
}

// Unified internal motion enqueue function
static void motion_enqueue_internal(pos_phys_t to_pos,
                                    movement_type_t move_type,
                                    float velocity,
                                    motion_stop_reason_t conditions,
                                    bool is_homing,
                                    axis_t home_axis) {
  comm_assert(state != MOTION_STATE_MOVING);

  // Initialize path buffer with single segment
  pb_init(&motion_path, &pos, &to_pos);
  latest_edm_state.distance_max = 0;

  // Set movement type and velocity
  movement_type = move_type;
  movement_velocity = velocity;

  // Set stop conditions
  stop_conditions = conditions;

  // Set homing state
  homing = is_homing;
  if (is_homing) {
    homing_axis = home_axis;
  }

  // Start moving
  motion_stats.start++;
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
                          STOP_REASON_TARGET, false, AXIS_X);
}

void motion_start_edm_move(pos_phys_t to_pos, bool has_cont) {
  edm_current_speed = EDM_INITIAL_VELOCITY_MM_PER_S;
  motion_enqueue_internal(to_pos, MOVEMENT_EDM_CONTROL, 0.0f,
                          has_cont ? STOP_REASON_NONE : STOP_REASON_TARGET,
                          false, AXIS_X);
}

void motion_start_probe_move(pos_phys_t to_pos) {
  motion_enqueue_internal(
      to_pos, MOVEMENT_CONSTANT_VELOCITY, PROBE_VELOCITY_MM_PER_S,
      STOP_REASON_TARGET | STOP_REASON_PROBE, false, AXIS_X);
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
                          VELOCITY_MM_PER_S,
                          STOP_REASON_TARGET | STOP_REASON_STALL, true, axis);
}

bool motion_move_can_enqueue() {
  unsigned int key = irq_lock();
  bool can_enqueue = pb_can_write(&motion_path);
  irq_unlock(key);
  return can_enqueue;
}

void motion_move_enqueue_pos(pos_phys_t to_pos, bool has_cont) {
  unsigned int key = irq_lock();
  if (!has_cont) {
    stop_conditions |= STOP_REASON_TARGET;
  }
  comm_assert(pb_can_write(&motion_path));
  pb_write(&motion_path, &to_pos);
  irq_unlock(key);
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

void motion_dump_status() {
  unsigned int key = irq_lock();
  motion_stat_cnt_t stat = motion_stats;
  irq_unlock(key);

  comm_ps_k_vint(PS_STAT, "motion.n_start", stat.start);
  comm_ps_k_vint(PS_STAT, "motion.n_stop", stat.stop);
  comm_ps_k_vint(PS_STAT, "motion.n_stop_probe", stat.stop_probe);
  comm_ps_k_vint(PS_STAT, "motion.n_stop_stall", stat.stop_stall);
  comm_ps_k_vint(PS_STAT, "motion.n_stop_target", stat.stop_target);
}
