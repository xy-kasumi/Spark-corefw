// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "gcode.h"

#include "canceler.h"
#include "comm.h"
#include "coords.h"
#include "gcode_base.h"
#include "motion.h"
#include "pulser.h"
#include "toolsupply.h"
#include "wirefeed.h"

#include <stdlib.h>
#include <zephyr/kernel.h>

static bool last_has_cont = false;

// Modal state for coordinate systems
static coord_system_t current_coord_system = COORD_SYSTEM_MACHINE;
static coord_offsets_t coord_offsets = {0};

// Last target position specified by G-code, in current_coord_system.
// Not available when canceled.
static bool last_target_avail = true;
static pos_phys_t last_target;

// Home phase configuration (pushed from settings) - X, Y, Z only
static int home_phases[3] = {0, 0, 0};

// Pulser configuration state
typedef struct {
  bool tool_negative;
  float pulse_us;
  float current_a;
  float duty_pct;
} pulser_config_t;

static pulser_config_t pulser_config = {.tool_negative = true,
                                        .pulse_us = 500.0f,
                                        .current_a = 1.0f,
                                        .duty_pct = 25.0f};

typedef struct {
  axis_t axis;
  int phase;
} axis_phase_t;

static int compare_axis_phase(const void* a, const void* b) {
  const axis_phase_t* axis_a = (const axis_phase_t*)a;
  const axis_phase_t* axis_b = (const axis_phase_t*)b;
  return axis_a->phase - axis_b->phase;
}

// Get "base pos" in current_coord_system.
// This is used for partial commands like "G1 X10" to get other axes values.
static pos_phys_t get_base_pos() {
  if (last_target_avail) {
    return last_target;
  } else {
    pos_phys_t machine_pos = motion_get_current_pos();
    return coords_from_machine(&machine_pos, current_coord_system,
                               &coord_offsets);
  }
}

/**
 * Validate whether block has valid move (e.g. G0, G1, G38.2) specification
 * parameters.
 * @returns true if valid, false if invalid. If invalid, comm_error() will be
 * called.
 */
static bool validate_move_spec(slice_t block, const gcode_parsed_t* parsed) {
  if (parsed->x_state == AXIS_ONLY || parsed->y_state == AXIS_ONLY ||
      parsed->z_state == AXIS_ONLY || parsed->c_state == AXIS_ONLY) {
    comm_error(block, "missing axis value");
    return false;
  }
  if (parsed->x_state == AXIS_NOT_SPECIFIED &&
      parsed->y_state == AXIS_NOT_SPECIFIED &&
      parsed->z_state == AXIS_NOT_SPECIFIED &&
      parsed->c_state == AXIS_NOT_SPECIFIED) {
    comm_error(block, "1 or more axes needed");
    return false;
  }
  return true;
}

/**
 * Get move target position. Does not update last_target or last_target_avail.
 *
 * @param parsed must be validated by validate_move_spec() first.
 * @returns target position in current_coord_system.
 */
static pos_phys_t get_move_target(const gcode_parsed_t* parsed) {
  pos_phys_t target_pos = get_base_pos();
  if (parsed->x_state == AXIS_WITH_VALUE) {
    target_pos.x = parsed->x;
  }
  if (parsed->y_state == AXIS_WITH_VALUE) {
    target_pos.y = parsed->y;
  }
  if (parsed->z_state == AXIS_WITH_VALUE) {
    target_pos.z = parsed->z;
  }
  if (parsed->c_state == AXIS_WITH_VALUE) {
    target_pos.c = parsed->c / 360.0f;  // Convert degrees to turns
  }
  return target_pos;
}

/**
 * Prepares a move command. If ok, last_target will be updated.
 *
 * @param block context of this command
 * @param parsed non-validated g-code
 * @param pos_machine output target position in machine coordinates
 * @returns true if valid, false if invalid. If invalid, comm_error() will be
 * called.
 */
static bool prepare_move(slice_t block,
                         const gcode_parsed_t* parsed,
                         pos_phys_t* pos_machine) {
  if (!validate_move_spec(block, parsed)) {
    return false;
  }
  pos_phys_t targ = get_move_target(parsed);
  pos_phys_t targ_machine =
      coords_to_machine(&targ, current_coord_system, &coord_offsets);

  last_target = targ;
  last_target_avail = true;
  *pos_machine = targ_machine;

  return true;
}

// Get axes sorted by home phase (lower phase = earlier)
static void get_home_order(axis_t result[3]) {
  axis_phase_t axes[3] = {{AXIS_X, home_phases[AXIS_X]},
                          {AXIS_Y, home_phases[AXIS_Y]},
                          {AXIS_Z, home_phases[AXIS_Z]}};

  qsort(axes, 3, sizeof(axis_phase_t), compare_axis_phase);

  for (int i = 0; i < 3; i++) {
    result[i] = axes[i].axis;
  }
}

/**
 * Wait until current "move" command ends.
 * End means either:
 * - canceled
 * - (if !cont_next)reached end condition
 * - (if cont_next) motion queue become available for next write.
 */
static void wait_move_command_end(bool cont_next) {
  while (!canceler_cancel_needed()) {
    if (cont_next) {
      if (motion_move_can_enqueue()) {
        break;
      }
    } else {
      if (motion_is_stopped(NULL)) {
        break;
      }
    }
    k_sleep(K_MSEC(10));
  }

  if (!cont_next) {
    pulser_deenergize();
  }
}

static void exec_gcode_cmd(slice_t block,
                           const gcode_parsed_t* parsed,
                           bool cont_prev,
                           bool cont_next) {
  if (parsed->code == 0 && parsed->sub_code == -1) {
    // G0 - rapid positioning
    pos_phys_t targ_machine;
    if (!prepare_move(block, parsed, &targ_machine)) {
      return;
    }
    motion_start_fast_move(targ_machine);
    wait_move_command_end(cont_next);
  } else if (parsed->code == 1 && parsed->sub_code == -1) {
    // G1 - controlled EDM move
    pos_phys_t targ_machine;
    if (!prepare_move(block, parsed, &targ_machine)) {
      return;
    }
    if (cont_prev) {
      motion_move_enqueue_pos(targ_machine, cont_next);
    } else {
      pulser_energize(pulser_config.tool_negative, pulser_config.pulse_us,
                      pulser_config.current_a, pulser_config.duty_pct);
      motion_start_edm_move(targ_machine, cont_next);
    }
    wait_move_command_end(cont_next);
  } else if (parsed->code == 28 && parsed->sub_code == -1) {
    // G28 - homing
    // Validate: no axis specified (home all) or exactly one axis (X, Y, Z only)
    bool x_specified = (parsed->x_state == AXIS_ONLY);
    bool y_specified = (parsed->y_state == AXIS_ONLY);
    bool z_specified = (parsed->z_state == AXIS_ONLY);
    bool c_specified = (parsed->c_state == AXIS_ONLY);
    int axis_count = x_specified + y_specified + z_specified + c_specified;

    if (c_specified) {
      comm_error(block, "C homing not supported");
      return;
    }

    if (axis_count == 0) {
      // Home all axes in phase order
      axis_t home_order[3];
      get_home_order(home_order);

      for (int i = 0; i < 3; i++) {
        motion_start_home(home_order[i]);
        while (!canceler_cancel_needed() && !motion_is_stopped(NULL)) {
          k_sleep(K_MSEC(10));
        }

        if (canceler_cancel_needed()) {
          break;
        }
      }
    } else if (axis_count == 1) {
      // Home specified axis
      axis_t target_axis = x_specified ? AXIS_X : y_specified ? AXIS_Y : AXIS_Z;
      motion_start_home(target_axis);
      while (!canceler_cancel_needed() && !motion_is_stopped(NULL)) {
        k_sleep(K_MSEC(10));
      }
    } else {
      comm_error(block, "too many axes");
    }
  } else if (parsed->code == 38 && parsed->sub_code == 3) {
    // G38.3 - probe towards target, no error when not found.
    pos_phys_t targ_machine;
    if (!prepare_move(block, parsed, &targ_machine)) {
      return;
    }
    pulser_energize(pulser_config.tool_negative, pulser_config.pulse_us,
                    pulser_config.current_a, pulser_config.duty_pct);
    motion_start_probe_move(targ_machine);
    wait_move_command_end(cont_next);
  } else if (parsed->code == 53 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_MACHINE;
  } else if (parsed->code == 54 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_GRINDER;
  } else if (parsed->code == 55 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_WORK;
  } else if (parsed->code == 56 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_TOOLSUPPLY;
  } else {
    comm_error(block, "unknown G-code");
  }
}

/**
 * Compute pulser_config_t from M3/M4 parameters.
 */
static pulser_config_t decode_pulser_params(bool tool_negative,
                                            const gcode_parsed_t* parsed) {
  pulser_config_t conf;
  conf.tool_negative = tool_negative;
  conf.pulse_us = (parsed->p_state == PARAM_SPECIFIED)
                      ? parsed->p
                      : 500.0f;  // Default 500us
  conf.current_a =
      (parsed->q_state == PARAM_SPECIFIED) ? parsed->q : 1.0f;  // Default 1A
  conf.duty_pct =
      (parsed->r_state == PARAM_SPECIFIED) ? parsed->r : 25.0f;  // Default 25%
  return conf;
}

static void exec_mcode_cmd(slice_t block, const gcode_parsed_t* parsed) {
  if (parsed->code == 3 && parsed->sub_code == -1) {
    pulser_config = decode_pulser_params(true, parsed);
  } else if (parsed->code == 4 && parsed->sub_code == -1) {
    pulser_config = decode_pulser_params(false, parsed);
  } else if (parsed->code == 10 && parsed->sub_code == -1) {
    // M10 - Start wire feeding
    if (parsed->r_state != PARAM_SPECIFIED) {
      comm_error(block, "R (feed[mm/min]) required");
      return;
    }
    wirefeed_start(parsed->r);
    // Wait 2 seconds for tension stabilization
    k_sleep(K_MSEC(2000));
  } else if (parsed->code == 11 && parsed->sub_code == -1) {
    wirefeed_stop();
  } else if (parsed->code == 60 && parsed->sub_code == -1) {
    set_tool_supply_state(TOOL_SUPPLY_OPEN);
  } else if (parsed->code == 61 && parsed->sub_code == -1) {
    set_tool_supply_state(TOOL_SUPPLY_CLOSED);
  } else {
    comm_error(block, "unknown M-code");
  }
}

static bool is_g1(const gcode_parsed_t* parsed) {
  return (parsed->cmd_type == CMD_TYPE_G && parsed->code == 1 &&
          parsed->sub_code == -1);
}

void exec_gcode(slice_t block, slice_t maybe_next_block) {
  gcode_parsed_t parsed;
  if (!parse_gcode(block, &parsed)) {
    comm_error(block, "syntax error");
    return;
  }

  // Detect continuation condition.
  bool cont_next = false;
  if (!sl_is_empty(maybe_next_block)) {
    gcode_parsed_t parsed_next;
    if (parse_gcode(maybe_next_block, &parsed_next)) {
      cont_next = is_g1(&parsed) && is_g1(&parsed_next);
    }
  }

  if (parsed.cmd_type == CMD_TYPE_G) {
    exec_gcode_cmd(block, &parsed, last_has_cont, cont_next);
  } else if (parsed.cmd_type == CMD_TYPE_M) {
    exec_mcode_cmd(block, &parsed);
  }

  if (canceler_cancel_needed()) {
    last_has_cont = false;
    last_target_avail = false;
  } else {
    last_has_cont = cont_next;
  }
}

coord_system_t gcode_get_current_coord_system() {
  return current_coord_system;
}

void gcode_set_coord_offset(coord_system_t cs_type, axis_t axis, float value) {
  pos_phys_t* target_origin = NULL;

  if (cs_type == COORD_SYSTEM_GRINDER) {
    target_origin = &coord_offsets.grinder_origin;
  } else if (cs_type == COORD_SYSTEM_WORK) {
    target_origin = &coord_offsets.work_origin;
  } else if (cs_type == COORD_SYSTEM_TOOLSUPPLY) {
    target_origin = &coord_offsets.toolsupply_origin;
  } else {
    return;  // Invalid coordinate system
  }

  switch (axis) {
    case AXIS_X:
      target_origin->x = value;
      break;
    case AXIS_Y:
      target_origin->y = value;
      break;
    case AXIS_Z:
      target_origin->z = value;
      break;
    case AXIS_C:
      target_origin->c = value / 360.0f;  // Convert degrees to turns
      break;
    default:
      // Invalid axis, do nothing
      break;
  }
}

const coord_offsets_t* gcode_get_coord_offsets() {
  return &coord_offsets;
}

void gcode_set_home_phase(axis_t axis, int phase) {
  if (axis == AXIS_X || axis == AXIS_Y || axis == AXIS_Z) {
    home_phases[axis] = phase;
  }
}

void exec_test_pulser(int dur_sec) {
  pulser_energize(pulser_config.tool_negative, pulser_config.pulse_us,
                  pulser_config.current_a, pulser_config.duty_pct);

  for (int i = 0; i < dur_sec * 10; i++) {
    if (canceler_cancel_needed()) {
      break;
    }

    // Print rates every second (every 10 iterations)
    if (i > 0 && i % 10 == 0) {
      // uint8_t short_rate = pulser_get_short_rate();
      // uint8_t pulse_rate = pulser_get_pulse_rate();
      // uint8_t open_rate = pulser_get_open_rate();
      // CM:comm_print("pulser short=%.2f pulse=%.2f open=%.2f", short_rate /
      // 255.0,pulse_rate / 255.0, open_rate / 255.0);
    }

    k_sleep(K_MSEC(100));
  }

  // De-energize pulser
  pulser_deenergize();
}
