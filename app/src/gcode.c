// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "gcode.h"

#include "comm.h"
#include "gcode_base.h"
#include "motion.h"
#include "pulser.h"
#include "system.h"
#include "wirefeed.h"

#include <zephyr/kernel.h>

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
    pos_phys_t p = motion_get_current_pos();
    if (parsed->x_state == AXIS_WITH_VALUE) {
      p.x = parsed->x;
    }
    if (parsed->y_state == AXIS_WITH_VALUE) {
      p.y = parsed->y;
    }
    if (parsed->z_state == AXIS_WITH_VALUE) {
      p.z = parsed->z;
    }
    motion_enqueue_move(p);
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
    pos_phys_t p = motion_get_current_pos();
    if (parsed->x_state == AXIS_WITH_VALUE) {
      p.x = parsed->x;
    }
    if (parsed->y_state == AXIS_WITH_VALUE) {
      p.y = parsed->y;
    }
    if (parsed->z_state == AXIS_WITH_VALUE) {
      p.z = parsed->z;
    }
    motion_enqueue_edm_move(p);
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
      comm_print("motion cancelled (pulser de-energized for safety)");
      pulser_deenergize();  // for safety
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
