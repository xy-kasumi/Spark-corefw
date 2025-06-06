#include "motion.h"

#include "comm.h"
#include "motor.h"

#include <math.h>
#include <zephyr/kernel.h>

// Motion constants
static const float VELOCITY_MM_PER_S = 10.0f;

// Motor configuration (pushed from settings)
static float motor_unitsteps[3] = {200.0f, 200.0f, 200.0f};

// Convert physical position to driver coordinates (microsteps)
static pos_drv_t phys_to_drv(pos_phys_t phys) {
  return (pos_drv_t){
      .m0 = (int)(phys.x * motor_unitsteps[0]),   // X maps to motor 0
      .m1 = (int)(phys.y * motor_unitsteps[1]),   // Y maps to motor 1
      .m2 = (int)(phys.z * motor_unitsteps[2])};  // Z maps to motor 2
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
static float move_duration;  // Total duration in seconds

// Timer for periodic tick
static struct k_timer motion_timer;

static void motion_tick_handler(struct k_timer* timer) {
  if (state != MOTION_STATE_MOVING) {
    return;
  }

  // Update progress (1ms = 0.001s)
  move_progress += VELOCITY_MM_PER_S * 0.001f;

  if (move_progress >= move_distance) {
    // Move completed
    pos = target_pos;
    state = MOTION_STATE_STOPPED;

    // De-energize motors
    motor_energize(0, false);
    motor_energize(1, false);
    motor_energize(2, false);

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
  move_duration = move_distance / VELOCITY_MM_PER_S;

  // Energize motors
  motor_energize(0, true);
  motor_energize(1, true);
  motor_energize(2, true);

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
