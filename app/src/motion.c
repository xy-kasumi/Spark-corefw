#include "motion.h"

#include "motor.h"

static pos_phys_t pos;
static motion_state_t state = MOTION_STATE_STOPPED;

void motion_tick_handler() {
  // TODO: do update. send pos to motors.
}

void motion_init() {
  // TODO: start periodic tick
}

pos_phys_t motion_get_current_pos() {
  return pos;
}

void motion_enqueue_move(pos_phys_t to_pos) {
  pos = to_pos;
  state = MOTION_STATE_STOPPED;
}

motion_state_t motion_get_current_state() {
  return state;
}
