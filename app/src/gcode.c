// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "gcode.h"

#include "comm.h"
#include "coords.h"
#include "gcode_base.h"
#include "motion.h"
#include "pulser.h"
#include "settings.h"
#include "system.h"
#include "wirefeed.h"

#include <zephyr/kernel.h>

// Modal state for coordinate systems
static coord_system_t current_coord_system = COORD_SYSTEM_MACHINE;
static coord_offsets_t coord_offsets = {0};

static void exec_gcode_cmd(const gcode_parsed_t* parsed) {
  if (parsed->code == 0 && parsed->sub_code == -1) {
    // G0 - rapid positioning
    // Validate: requires AXIS_WITH_VALUE, not AXIS_ONLY, and at least one axis
    if (parsed->x_state == AXIS_ONLY || parsed->y_state == AXIS_ONLY ||
        parsed->z_state == AXIS_ONLY) {
      comm_print_err("G0 requires axis values (e.g., X10.5), not bare axes");
      return;
    }
    if (parsed->x_state == AXIS_NOT_SPECIFIED &&
        parsed->y_state == AXIS_NOT_SPECIFIED &&
        parsed->z_state == AXIS_NOT_SPECIFIED) {
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

    // Convert back to machine coordinates for motion system
    pos_phys_t machine_target =
        coords_to_machine(&target_pos, current_coord_system, &coord_offsets);
    motion_enqueue_move(machine_target);
  } else if (parsed->code == 1 && parsed->sub_code == -1) {
    // G1 - controlled EDM move
    // Same validation as G0
    if (parsed->x_state == AXIS_ONLY || parsed->y_state == AXIS_ONLY ||
        parsed->z_state == AXIS_ONLY) {
      comm_print_err("G1 requires axis values (e.g., X10.5), not bare axes");
      return;
    }
    if (parsed->x_state == AXIS_NOT_SPECIFIED &&
        parsed->y_state == AXIS_NOT_SPECIFIED &&
        parsed->z_state == AXIS_NOT_SPECIFIED) {
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

    // Convert back to machine coordinates for motion system
    pos_phys_t machine_target =
        coords_to_machine(&target_pos, current_coord_system, &coord_offsets);
    motion_enqueue_edm_move(machine_target);
  } else if (parsed->code == 28 && parsed->sub_code == -1) {
    // G28 - homing
    // Validate: requires exactly one axis with AXIS_ONLY format
    bool x_specified = (parsed->x_state == AXIS_ONLY);
    bool y_specified = (parsed->y_state == AXIS_ONLY);
    bool z_specified = (parsed->z_state == AXIS_ONLY);
    int axis_count = x_specified + y_specified + z_specified;

    if (axis_count != 1) {
      comm_print_err(
          "G28 requires exactly one axis without value (X, Y, or Z)");
      return;
    }

    // Execute: home the specified axis
    if (x_specified) {
      motion_enqueue_home(0);  // Home X axis
    } else if (y_specified) {
      motion_enqueue_home(1);  // Home Y axis
    } else if (z_specified) {
      motion_enqueue_home(2);  // Home Z axis
    }
  } else if (parsed->code == 53 && parsed->sub_code == -1) {
    // G53 - Use machine coordinate system
    current_coord_system = COORD_SYSTEM_MACHINE;
    comm_print("coordinate system: machine");
    return;  // No motion, return early
  } else if (parsed->code == 54 && parsed->sub_code == -1) {
    // G54 - Use grinder coordinate system
    current_coord_system = COORD_SYSTEM_GRINDER;
    comm_print("coordinate system: grinder");
    return;  // No motion, return early
  } else if (parsed->code == 55 && parsed->sub_code == -1) {
    // G55 - Use work coordinate system
    current_coord_system = COORD_SYSTEM_WORK;
    comm_print("coordinate system: work");
    return;  // No motion, return early
  } else {
    if (parsed->sub_code != -1) {
      comm_print_err("Unsupported G-code: G%d.%d", parsed->code,
                     parsed->sub_code);
    } else {
      comm_print_err("Unsupported G-code: G%d", parsed->code);
    }
    return;
  }

  // Wait for motion completion
  while (true) {
    if (motion_get_current_state() == MOTION_STATE_STOPPED) {
      break;
    }
    k_sleep(K_MSEC(10));
  }
  switch (motion_get_last_stop_reason()) {
    case STOP_REASON_TARGET_REACHED:
      comm_print("motion completed");
      break;
    case STOP_REASON_STALL_DETECTED:
      comm_print("stall detected");
      break;
    case STOP_REASON_PROBE_TRIGGERED:
      comm_print("probe triggered");
      break;
    case STOP_REASON_CANCELLED:
      comm_print(
          "motion cancelled (for safety, pulser de-energized & wirefeed "
          "stopped)");
      pulser_deenergize();  // for safety
      wirefeed_stop();      // for safety
      break;
    default:
      comm_print_err("motion ended for unknown reason");
      break;
  }
}

static void exec_mcode_cmd(const gcode_parsed_t* parsed) {
  if (parsed->code == 3 && parsed->sub_code == -1) {
    // M3 - Energize, tool negative voltage
    // Validate: P (pulse time), Q (current), R (duty) are optional
    float pulse_time_us = (parsed->p_state == PARAM_SPECIFIED)
                              ? parsed->p
                              : 500.0f;  // Default 500us
    float pulse_current_a =
        (parsed->q_state == PARAM_SPECIFIED) ? parsed->q : 1.0f;  // Default 1A
    float max_duty_pct = (parsed->r_state == PARAM_SPECIFIED)
                             ? parsed->r
                             : 25.0f;  // Default 25%

    pulser_energize(true, pulse_time_us, pulse_current_a, max_duty_pct);
  } else if (parsed->code == 4 && parsed->sub_code == -1) {
    // M4 - Energize, tool positive voltage
    // Validate: P (pulse time), Q (current), R (duty) are optional
    float pulse_time_us = (parsed->p_state == PARAM_SPECIFIED)
                              ? parsed->p
                              : 500.0f;  // Default 500us
    float pulse_current_a =
        (parsed->q_state == PARAM_SPECIFIED) ? parsed->q : 1.0f;  // Default 1A
    float max_duty_pct = (parsed->r_state == PARAM_SPECIFIED)
                             ? parsed->r
                             : 25.0f;  // Default 25%

    pulser_energize(false, pulse_time_us, pulse_current_a, max_duty_pct);
  } else if (parsed->code == 5 && parsed->sub_code == -1) {
    // M5 - De-energize
    pulser_deenergize();
  } else if (parsed->code == 10 && parsed->sub_code == -1) {
    // M10 - Start wire feeding
    if (parsed->r_state != PARAM_SPECIFIED) {
      comm_print_err("M10 requires R parameter (feed rate in mm/min)");
      return;
    }
    wirefeed_start(parsed->r);
  } else if (parsed->code == 11 && parsed->sub_code == -1) {
    // M11 - Stop wire feeding
    wirefeed_stop();
  } else {
    comm_print_err("Unsupported M-code: M%d", parsed->code);
  }
}

void exec_gcode(char* full_command) {
  gcode_parsed_t parsed;
  if (!parse_gcode(full_command, &parsed)) {
    comm_print_err("Failed to parse G/M-code: %s", full_command);
    return;
  }

  if (parsed.cmd_type == CMD_TYPE_G) {
    exec_gcode_cmd(&parsed);
  } else if (parsed.cmd_type == CMD_TYPE_M) {
    exec_mcode_cmd(&parsed);
  }
}

coord_system_t gcode_get_current_coord_system() {
  return current_coord_system;
}

void gcode_set_coord_offset(coord_system_t cs_type, int axis, float value) {
  pos_phys_t* target_origin = NULL;

  if (cs_type == COORD_SYSTEM_GRINDER) {
    target_origin = &coord_offsets.grinder_origin;
  } else if (cs_type == COORD_SYSTEM_WORK) {
    target_origin = &coord_offsets.work_origin;
  } else {
    return;  // Invalid coordinate system
  }

  switch (axis) {
    case 0:
      target_origin->x = value;
      break;
    case 1:
      target_origin->y = value;
      break;
    case 2:
      target_origin->z = value;
      break;
    default:
      // Invalid axis, do nothing
      break;
  }
}

const coord_offsets_t* gcode_get_coord_offsets() {
  return &coord_offsets;
}
