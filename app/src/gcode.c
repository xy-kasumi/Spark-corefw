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

static void exec_gcode_cmd(const gcode_parsed_t* parsed,
                           bool cont_prev,
                           bool cont_next) {
  if (parsed->code == 0 && parsed->sub_code == -1) {
    // G0 - rapid positioning
    // Validate: requires AXIS_WITH_VALUE, not AXIS_ONLY, and at least one axis
    if (parsed->x_state == AXIS_ONLY || parsed->y_state == AXIS_ONLY ||
        parsed->z_state == AXIS_ONLY || parsed->c_state == AXIS_ONLY) {
      comm_print_err("G0 requires axis values (e.g., X10.5), not bare axes");
      return;
    }
    if (parsed->x_state == AXIS_NOT_SPECIFIED &&
        parsed->y_state == AXIS_NOT_SPECIFIED &&
        parsed->z_state == AXIS_NOT_SPECIFIED &&
        parsed->c_state == AXIS_NOT_SPECIFIED) {
      comm_print_err("G0 requires at least one axis parameter");
      return;
    }

    // Execute: move to specified coordinates
    // Get current position in machine coordinates
    pos_phys_t machine_pos = motion_get_current_pos();
    // Convert to current coordinate system for updating
    pos_phys_t target_pos =
        coords_from_machine(&machine_pos, current_coord_system, &coord_offsets);

    // Update with parsed values
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

    // Convert back to machine coordinates for motion system
    pos_phys_t machine_target =
        coords_to_machine(&target_pos, current_coord_system, &coord_offsets);

    motion_start_fast_move(machine_target);

    while (true) {
      if (canceler_cancel_needed()) {
        pulser_deenergize();
        break;
      }
      if (motion_is_stopped(NULL)) {
        pulser_deenergize();
        break;
      }
      k_sleep(K_MSEC(10));
    }
  } else if (parsed->code == 1 && parsed->sub_code == -1) {
    // G1 - controlled EDM move
    // Same validation as G0
    if (parsed->x_state == AXIS_ONLY || parsed->y_state == AXIS_ONLY ||
        parsed->z_state == AXIS_ONLY || parsed->c_state == AXIS_ONLY) {
      comm_print_err("G1 requires axis values (e.g., X10.5), not bare axes");
      return;
    }
    if (parsed->x_state == AXIS_NOT_SPECIFIED &&
        parsed->y_state == AXIS_NOT_SPECIFIED &&
        parsed->z_state == AXIS_NOT_SPECIFIED &&
        parsed->c_state == AXIS_NOT_SPECIFIED) {
      comm_print_err("G1 requires at least one axis parameter");
      return;
    }

    // Execute: EDM move to specified coordinates
    // Get current position in machine coordinates
    pos_phys_t machine_pos = motion_get_current_pos();
    // Convert to current coordinate system for updating
    pos_phys_t target_pos =
        coords_from_machine(&machine_pos, current_coord_system, &coord_offsets);

    // Update with parsed values
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

    // Convert back to machine coordinates for motion system
    pos_phys_t machine_target =
        coords_to_machine(&target_pos, current_coord_system, &coord_offsets);

    if (cont_prev) {
      motion_move_enqueue_pos(machine_target, cont_next);
    } else {
      pulser_energize(pulser_config.tool_negative, pulser_config.pulse_us,
                      pulser_config.current_a, pulser_config.duty_pct);
      motion_start_edm_move(machine_target, cont_next);
    }

    // Wait until next g-code block become executable.
    if (cont_next) {
      while (true) {
        if (canceler_cancel_needed()) {
          break;
        }
        if (motion_move_can_enqueue()) {
          break;
        }
        k_sleep(K_MSEC(10));
      }
    } else {
      while (true) {
        if (canceler_cancel_needed()) {
          pulser_deenergize();
          break;
        }
        if (motion_is_stopped(NULL)) {
          pulser_deenergize();
          break;
        }
        k_sleep(K_MSEC(10));
      }
    }
  } else if (parsed->code == 28 && parsed->sub_code == -1) {
    // G28 - homing
    // Validate: no axis specified (home all) or exactly one axis (X, Y, Z only)
    bool x_specified = (parsed->x_state == AXIS_ONLY);
    bool y_specified = (parsed->y_state == AXIS_ONLY);
    bool z_specified = (parsed->z_state == AXIS_ONLY);
    bool c_specified = (parsed->c_state == AXIS_ONLY);
    int axis_count = x_specified + y_specified + z_specified + c_specified;

    if (c_specified) {
      comm_print_err("G28 C not supported (C-axis has no home position)");
      return;
    }

    if (axis_count == 0) {
      // Home all axes in phase order
      axis_t home_order[3];
      get_home_order(home_order);

      for (int i = 0; i < 3; i++) {
        // CM:comm_print("homing axis %c", axis_to_letter(home_order[i]));
        motion_start_home(home_order[i]);

        // Wait for motion completion
        while (true) {
          if (canceler_cancel_needed()) {
            break;
          }
          if (motion_is_stopped(NULL)) {
            break;
          }
          k_sleep(K_MSEC(10));
        }

        if (canceler_cancel_needed()) {
          break;
        }
      }
    } else if (axis_count == 1) {
      // Execute: home the specified axis
      axis_t target_axis = x_specified ? AXIS_X : y_specified ? AXIS_Y : AXIS_Z;
      motion_start_home(target_axis);
      while (true) {
        if (motion_is_stopped(NULL)) {
          break;
        }
        k_sleep(K_MSEC(10));
      }
    } else {
      comm_print_err(
          "G28 requires no parameters (all axes) or exactly one axis");
    }
  } else if (parsed->code == 38 && parsed->sub_code == 3) {
    // G38.3 - probe towards target, no error
    // Same validation as G0/G1
    if (parsed->x_state == AXIS_ONLY || parsed->y_state == AXIS_ONLY ||
        parsed->z_state == AXIS_ONLY || parsed->c_state == AXIS_ONLY) {
      comm_print_err("G38.3 requires axis values (e.g., X10.5), not bare axes");
      return;
    }
    if (parsed->x_state == AXIS_NOT_SPECIFIED &&
        parsed->y_state == AXIS_NOT_SPECIFIED &&
        parsed->z_state == AXIS_NOT_SPECIFIED &&
        parsed->c_state == AXIS_NOT_SPECIFIED) {
      comm_print_err("G38.3 requires at least one axis parameter");
      return;
    }

    // Execute: probe move to specified coordinates
    // Get current position in machine coordinates
    pos_phys_t machine_pos = motion_get_current_pos();
    // Convert to current coordinate system for updating
    pos_phys_t target_pos =
        coords_from_machine(&machine_pos, current_coord_system, &coord_offsets);

    // Update with parsed values
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

    // Convert back to machine coordinates for motion system
    pos_phys_t machine_target =
        coords_to_machine(&target_pos, current_coord_system, &coord_offsets);

    // Energize pulser with current config (configured or defaults)
    pulser_energize(pulser_config.tool_negative, pulser_config.pulse_us,
                    pulser_config.current_a, pulser_config.duty_pct);

    motion_start_probe_move(machine_target);
    while (true) {
      if (canceler_cancel_needed()) {
        pulser_deenergize();
        break;
      }
      if (motion_is_stopped(NULL)) {
        pulser_deenergize();
        break;
      }
      k_sleep(K_MSEC(10));
    }
  } else if (parsed->code == 53 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_MACHINE;
  } else if (parsed->code == 54 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_GRINDER;
  } else if (parsed->code == 55 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_WORK;
  } else if (parsed->code == 56 && parsed->sub_code == -1) {
    current_coord_system = COORD_SYSTEM_TOOLSUPPLY;
  } else {
    if (parsed->sub_code != -1) {
      comm_print_err("Unsupported G-code: G%d.%d", parsed->code,
                     parsed->sub_code);
    } else {
      comm_print_err("Unsupported G-code: G%d", parsed->code);
    }
  }
}

static void exec_mcode_cmd(const gcode_parsed_t* parsed) {
  if (parsed->code == 3 && parsed->sub_code == -1) {
    // M3 - Configure EDM parameters, tool negative voltage
    // Validate: P (pulse time), Q (current), R (duty) are optional
    pulser_config.tool_negative = true;
    pulser_config.pulse_us = (parsed->p_state == PARAM_SPECIFIED)
                                 ? parsed->p
                                 : 500.0f;  // Default 500us
    pulser_config.current_a =
        (parsed->q_state == PARAM_SPECIFIED) ? parsed->q : 1.0f;  // Default 1A
    pulser_config.duty_pct = (parsed->r_state == PARAM_SPECIFIED)
                                 ? parsed->r
                                 : 25.0f;  // Default 25%
    // CM:comm_print("M3: pulser configured (T-, %.0fµs, %.1fA,
    // %.0f%%)",(double)pulser_config.pulse_us,
    // (double)pulser_config.current_a,(double)pulser_config.duty_pct);
  } else if (parsed->code == 4 && parsed->sub_code == -1) {
    // M4 - Configure EDM parameters, tool positive voltage
    // Validate: P (pulse time), Q (current), R (duty) are optional
    pulser_config.tool_negative = false;
    pulser_config.pulse_us = (parsed->p_state == PARAM_SPECIFIED)
                                 ? parsed->p
                                 : 500.0f;  // Default 500us
    pulser_config.current_a =
        (parsed->q_state == PARAM_SPECIFIED) ? parsed->q : 1.0f;  // Default 1A
    pulser_config.duty_pct = (parsed->r_state == PARAM_SPECIFIED)
                                 ? parsed->r
                                 : 25.0f;  // Default 25%
    // CM:comm_print("M4: pulser configured (T+, %.0fµs, %.1fA,
    // %.0f%%)",(double)pulser_config.pulse_us,
    // (double)pulser_config.current_a,(double)pulser_config.duty_pct);
  } else if (parsed->code == 10 && parsed->sub_code == -1) {
    // M10 - Start wire feeding
    if (parsed->r_state != PARAM_SPECIFIED) {
      comm_print_err("M10 requires R parameter (feed rate in mm/min)");
      return;
    }
    wirefeed_start(parsed->r);
    // Wait 2 seconds for tension stabilization
    k_sleep(K_MSEC(2000));
  } else if (parsed->code == 11 && parsed->sub_code == -1) {
    // M11 - Stop wire feeding
    wirefeed_stop();
  } else if (parsed->code == 60 && parsed->sub_code == -1) {
    // M60 - Open tool supply
    set_tool_supply_state(TOOL_SUPPLY_OPEN);
    // CM:comm_print("tool supply opened");
  } else if (parsed->code == 61 && parsed->sub_code == -1) {
    // M61 - Close tool supply
    set_tool_supply_state(TOOL_SUPPLY_CLOSED);
    // CM:comm_print("tool supply closed");
  } else {
    // CM:comm_print_err("Unsupported M-code: M%d", parsed->code);
  }
}

void exec_gcode(char* block, char* maybe_next_block) {
  gcode_parsed_t parsed;
  if (!parse_gcode(block, &parsed)) {
    // CM:comm_print_err("Failed to parse G/M-code: %s", full_command);
    return;
  }

  // Detect continuation condition.
  // Only continuation for now is G1->G1.
  bool cont_next = false;
  if (maybe_next_block) {
    gcode_parsed_t parsed_next;
    if (parse_gcode(maybe_next_block, &parsed_next)) {
      if (parsed.cmd_type == CMD_TYPE_G && parsed_next.cmd_type == CMD_TYPE_G) {
        if ((parsed.code == 1 && parsed.sub_code == -1) &&
            (parsed_next.code == 1 && parsed_next.sub_code == -1)) {
          cont_next = true;
        }
      }
    }
  }

  if (parsed.cmd_type == CMD_TYPE_G) {
    exec_gcode_cmd(&parsed, last_has_cont, cont_next);
  } else if (parsed.cmd_type == CMD_TYPE_M) {
    exec_mcode_cmd(&parsed);
  }

  if (canceler_cancel_needed()) {
    last_has_cont = false;
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
      // CM:comm_print("test pulser: cancelled");
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
