/**
 * Motion controller & coordinate management.
 */
#pragma once

/** Represents a single physical coordinate. (i.e. coordinates specification in
 * G-code)
 */
typedef struct {
  float x;
  float y;
  float z;
  // a, b, c etc.
} pos_phys_t;

/**
 * Represents motion state.
 */
typedef enum {
  MOTION_STATE_STOPPED,
  MOTION_STATE_MOVING,
} motion_state_t;

void motion_init();

pos_phys_t motion_get_current_pos();
void motion_enqueue_move(pos_phys_t to_pos);
motion_state_t motion_get_current_state();
