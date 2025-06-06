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

/** Compute distance between two pos_phys_t points.
 * @return distance in mm
 */
float posp_dist(const pos_phys_t* a, const pos_phys_t* b);

/** Linearly interpolate between a (t=0), and b (t=1).
 * t can be outside of [0, 1] range, in which case it will be extrapolated.
 */
void posp_interp(const pos_phys_t* a,
                 const pos_phys_t* b,
                 float t,
                 pos_phys_t* out);

void motion_init();

pos_phys_t motion_get_current_pos();
void motion_enqueue_move(pos_phys_t to_pos);
motion_state_t motion_get_current_state();

/** Set how many microsteps are needed for moving the corresponding axis in
 * +1unit (+1 mm or +1 rotation).
 *
 * TODO: Calling this after movement results in weird behavior.
 */
void motion_set_motor_unitsteps(int motor_num, float unitsteps);
