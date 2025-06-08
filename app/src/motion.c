#include "motion.h"

#include "comm.h"
#include "motor.h"
#include "system.h"

#include <drivers/tmc_driver.h>
#include <math.h>
#include <zephyr/kernel.h>

// Motion constants
static const float VELOCITY_MM_PER_S = 10.0f;

// Motor configuration (pushed from settings)
static float motor_unitsteps[3] = {200.0f, 200.0f, 200.0f};

// Home configuration (pushed from settings)
static float home_origins[3] = {0.0f, 0.0f, 0.0f};
static float home_sides[3] = {1.0f, 1.0f, 1.0f};

// Homing offset: bridges gap between driver coords and physical coords
// Updated after each successful home operation
static pos_drv_t homing_offset = {0, 0, 0};

// Constants
static const float MAX_TRAVEL_MM = 500.0f;

// Convert physical position to driver coordinates (microsteps)
static pos_drv_t phys_to_drv(pos_phys_t phys) {
  // Convert to raw driver steps
  pos_drv_t raw_drv = {
      .m0 = (int)(phys.x * motor_unitsteps[0]),   // X maps to motor 0
      .m1 = (int)(phys.y * motor_unitsteps[1]),   // Y maps to motor 1
      .m2 = (int)(phys.z * motor_unitsteps[2])};  // Z maps to motor 2

  // Apply homing offset to align with current coordinate system
  return (pos_drv_t){.m0 = raw_drv.m0 + homing_offset.m0,
                     .m1 = raw_drv.m1 + homing_offset.m1,
                     .m2 = raw_drv.m2 + homing_offset.m2};
}

// Update homing offset after successful homing
static void update_homing_offset(int axis) {
  // Get current driver position (where we actually are)
  pos_drv_t current_drv = motor_get_current_pos_drv();

  // Calculate where driver coordinates should be for the new physical origin
  pos_phys_t origin_phys = {home_origins[0], home_origins[1], home_origins[2]};
  pos_drv_t raw_expected = {.m0 = (int)(origin_phys.x * motor_unitsteps[0]),
                            .m1 = (int)(origin_phys.y * motor_unitsteps[1]),
                            .m2 = (int)(origin_phys.z * motor_unitsteps[2])};

  // Update offset for the homed axis so current driver position maps to origin
  if (axis == 0) {  // X axis
    homing_offset.m0 = current_drv.m0 - raw_expected.m0;
  } else if (axis == 1) {  // Y axis
    homing_offset.m1 = current_drv.m1 - raw_expected.m1;
  } else if (axis == 2) {  // Z axis
    homing_offset.m2 = current_drv.m2 - raw_expected.m2;
  }
}

// Helper functions
float posp_dist(const pos_phys_t* a, const pos_phys_t* b) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dz = b->z - a->z;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

void posp_interp(const pos_phys_t* a,
                 const pos_phys_t* b,
                 float t,
                 pos_phys_t* out) {
  out->x = a->x + (b->x - a->x) * t;
  out->y = a->y + (b->y - a->y) * t;
  out->z = a->z + (b->z - a->z) * t;
}

// Motion state
static pos_phys_t pos;
static motion_state_t state = MOTION_STATE_STOPPED;

// Motion planning state
static pos_phys_t target_pos;
static pos_phys_t start_pos;
static float move_distance;  // Total distance to move in mm
static float move_progress;  // Current progress in mm

// Stop condition flags
static bool stop_at_stall;
static bool stop_at_probe;
static motion_stop_reason_t last_stop_reason;
static int homing_axis;  // Which axis is being homed (-1 if not homing)

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
  if (stop_at_stall && homing_axis >= 0) {
    const struct device* motor = motor_get_device(homing_axis);
    if (motor && tmc_stalled(motor)) {
      // Homing completed - stall detected
      // Update homing offset before changing physical position
      update_homing_offset(homing_axis);

      // Set physical position to homing origin
      pos.x = (homing_axis == 0) ? home_origins[0] : pos.x;
      pos.y = (homing_axis == 1) ? home_origins[1] : pos.y;
      pos.z = (homing_axis == 2) ? home_origins[2] : pos.z;

      last_stop_reason = STOP_REASON_STALL_DETECTED;
      state = MOTION_STATE_STOPPED;
      return;
    }
  }

  // Update progress (1ms = 0.001s)
  move_progress += VELOCITY_MM_PER_S * 0.001f;

  if (move_progress >= move_distance) {
    // Move completed normally
    pos = target_pos;
    last_stop_reason = STOP_REASON_TARGET_REACHED;
    state = MOTION_STATE_STOPPED;
    return;
  }

  // Linear interpolation
  float t = move_progress / move_distance;  // 0.0 to 1.0
  posp_interp(&start_pos, &target_pos, t, &pos);

  // Convert to driver coordinates and send to motors
  pos_drv_t target_drv = phys_to_drv(pos);
  motor_set_target_pos_drv(target_drv);
}

void motion_init() {
  // Initialize timer for 1ms periodic tick
  k_timer_init(&motion_timer, motion_tick_handler, NULL);
  k_timer_start(&motion_timer, K_MSEC(1), K_MSEC(1));

  comm_print("Motion initialized with 1ms tick");
}

pos_phys_t motion_get_current_pos() {
  return pos;
}

void motion_enqueue_move(pos_phys_t to_pos) {
  // Don't start new move if already moving
  if (state == MOTION_STATE_MOVING) {
    return;
  }

  // Calculate move distance
  move_distance = posp_dist(&pos, &to_pos);

  // Skip if no movement needed
  if (move_distance < 0.001f) {
    return;
  }

  // Set up motion planning
  start_pos = pos;
  target_pos = to_pos;
  move_progress = 0.0f;

  // Clear stop conditions (normal move)
  stop_at_stall = false;
  stop_at_probe = false;
  homing_axis = -1;

  // Motors will auto-energize when they start moving

  // Start moving
  state = MOTION_STATE_MOVING;
}

motion_state_t motion_get_current_state() {
  return state;
}

void motion_set_motor_unitsteps(int motor_num, float unitsteps) {
  if (motor_num >= 0 && motor_num < 3) {
    motor_unitsteps[motor_num] = unitsteps;
  }
}

void motion_set_home_origin(int axis, float origin_mm) {
  if (axis >= 0 && axis < 3) {
    home_origins[axis] = origin_mm;
  }
}

void motion_set_home_side(int axis, float side) {
  if (axis >= 0 && axis < 3) {
    home_sides[axis] = side;
  }
}

motion_stop_reason_t motion_get_last_stop_reason() {
  return last_stop_reason;
}

void motion_enqueue_home(int axis) {
  // Don't start new move if already moving
  if (state == MOTION_STATE_MOVING) {
    return;
  }

  // Validate axis
  if (axis < 0 || axis >= 3) {
    return;
  }

  // Calculate target position for homing
  pos_phys_t home_target = pos;
  float side = home_sides[axis];
  if (axis == 0) {
    home_target.x += side * MAX_TRAVEL_MM;
  } else if (axis == 1) {
    home_target.y += side * MAX_TRAVEL_MM;
  } else if (axis == 2) {
    home_target.z += side * MAX_TRAVEL_MM;
  }

  // Calculate move distance
  move_distance = posp_dist(&pos, &home_target);

  // Skip if no movement needed
  if (move_distance < 0.001f) {
    return;
  }

  // Set up motion planning
  start_pos = pos;
  target_pos = home_target;
  move_progress = 0.0f;

  // Set stop conditions for homing
  stop_at_stall = true;
  stop_at_probe = false;
  homing_axis = axis;

  // Start homing
  state = MOTION_STATE_MOVING;
}
