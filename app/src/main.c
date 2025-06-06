// Application entry point for spark core firmware.
// Main command loop is executed here.
#include "comm.h"
#include "gcode.h"
#include "motion.h"
#include "motor.h"
#include "settings.h"
#include "strutil.h"
#include "system.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

// Command: help
static void cmd_help(char* args) {
  comm_print("help - Show this help");
  comm_print("stat motor - Show motor subsystem status");
  comm_print("steptest <motor_num> - Step motor test (0, 1, or 2)");
  comm_print("set <key> <value> - Set variable to value");
  comm_print("get - List all variables with values");
  comm_print("get <key> - Get specific variable value");
  comm_print("! - Cancel current operation");
}

// Command: gcode
static void cmd_gcode(char* full_command) {
  gcode_parsed_t parsed;
  if (!parse_gcode(full_command, &parsed)) {
    comm_print_err("Failed to parse G-code: %s", full_command);
    return;
  }

  if (parsed.code == 0 && parsed.sub_code == -1) {
    // G0 - rapid positioning
    // Validate: requires AXIS_WITH_VALUE, not AXIS_ONLY, and at least one axis
    if (parsed.x_state == AXIS_ONLY || parsed.y_state == AXIS_ONLY ||
        parsed.z_state == AXIS_ONLY) {
      comm_print_err("G0 requires axis values (e.g., X10.5), not bare axes");
      return;
    }
    if (parsed.x_state == AXIS_NOT_SPECIFIED &&
        parsed.y_state == AXIS_NOT_SPECIFIED &&
        parsed.z_state == AXIS_NOT_SPECIFIED) {
      comm_print_err("G0 requires at least one axis parameter");
      return;
    }

    // Execute: move to specified coordinates
    pos_phys_t p = motion_get_current_pos();
    if (parsed.x_state == AXIS_WITH_VALUE) {
      p.x = parsed.x;
    }
    if (parsed.y_state == AXIS_WITH_VALUE) {
      p.y = parsed.y;
    }
    if (parsed.z_state == AXIS_WITH_VALUE) {
      p.z = parsed.z;
    }
    motion_enqueue_move(p);
  } else if (parsed.code == 28 && parsed.sub_code == -1) {
    // G28 - homing
    // Validate: requires exactly one axis with AXIS_ONLY format
    bool x_specified = (parsed.x_state == AXIS_ONLY);
    bool y_specified = (parsed.y_state == AXIS_ONLY);
    bool z_specified = (parsed.z_state == AXIS_ONLY);
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
    comm_print_err("Unsupported g-code");
    return;
  }
  while (true) {
    motion_state_t state = motion_get_current_state();
    if (state == MOTION_STATE_STOPPED) {
      motion_stop_reason_t stop_reason = motion_get_last_stop_reason();
      switch (stop_reason) {
        case STOP_REASON_TARGET_REACHED:
          comm_print("Motion completed: target reached");
          break;
        case STOP_REASON_STALL_DETECTED:
          comm_print("Motion completed: stall detected");
          break;
        case STOP_REASON_PROBE_TRIGGERED:
          comm_print("Motion completed: probe triggered");
          break;
        default:
          comm_print("Motion completed: unknown reason");
          break;
      }
      break;
    }
    k_sleep(K_MSEC(10));
  }
}

// Command: set <key> <value>
static void cmd_set(char* args) {
  if (!args) {
    comm_print_err("Usage: set <key> <value>");
    return;
  }

  // Destructive parse: split key and value
  char* key = args;
  char* value = split_at(key, ' ');

  if (!value) {
    comm_print_err("Usage: set <key> <value>");
    return;
  }

  // Parse and validate float value
  float float_value;
  if (!parse_float(value, &float_value)) {
    comm_print_err("Invalid number: %s", value);
    return;
  }

  if (!settings_set(key, float_value)) {
    comm_print_err("Failed to set %s", key);
  }
}

// Command: get [key]
static void cmd_get(char* args) {
  if (!args || strlen(args) == 0) {
    // List all settings
    const char* key;
    float value;
    for (int i = 0; settings_get_by_index(i, &key, &value); i++) {
      comm_print("%s %.1f", key, (double)value);
    }
  } else {
    // Get specific setting - check if key exists
    bool key_exists = false;
    const char* test_key;
    for (int i = 0; settings_get_by_index(i, &test_key, NULL); i++) {
      if (strcmp(test_key, args) == 0) {
        key_exists = true;
        break;
      }
    }

    if (key_exists) {
      float value = settings_get(args);
      comm_print("%.1f", (double)value);
    } else {
      comm_print_err("Unknown key %s", args);
    }
  }
}

// Command: stat
static void cmd_stat(char* args) {
  if (!args || strlen(args) == 0) {
    comm_print_err("Usage: stat <subsystem>");
    comm_print("Available subsystems: motor");
    return;
  }

  if (strcmp(args, "motor") == 0) {
    motor_dump_status();
  } else {
    comm_print_err("Unknown subsystem: %s", args);
  }
}

// Command: steptest
static void cmd_steptest(char* args) {
  if (!args || strlen(args) == 0) {
    comm_print_err("Usage: steptest <motor_num>");
    return;
  }

  int motor_num;
  if (!parse_int(args, &motor_num)) {
    comm_print_err("Invalid motor number: %s", args);
    return;
  }

  motor_run_steptest(motor_num);
}

static void handle_console_command(char* command) {
  g_machine_state = STATE_EXEC_INTERACTIVE;
  comm_print_ack();

  // Check for G-code before destructive parsing
  if (command[0] == 'G') {
    cmd_gcode(command);  // Pass full command for G-code parsing
    goto cleanup;
  }

  // Destructive parse: split command and arguments
  char* cmd = command;
  char* args = split_at(cmd, ' ');

  // Dispatch to command handler
  if (strcmp(cmd, "help") == 0) {
    cmd_help(args);
  } else if (strcmp(cmd, "stat") == 0) {
    cmd_stat(args);
  } else if (strcmp(cmd, "steptest") == 0) {
    cmd_steptest(args);
  } else if (strcmp(cmd, "set") == 0) {
    cmd_set(args);
  } else if (strcmp(cmd, "get") == 0) {
    cmd_get(args);
  } else {
    comm_print_err("unknown command: %s; type 'help' for available commands",
                   cmd);
  }

cleanup:
  // TODO: verify motion state is MOTION_STATE_STOPPED.

  // Clear cancel flag and return to IDLE
  g_cancel_requested = false;
  g_machine_state = STATE_IDLE;

  // Print ready with current position
  pos_phys_t current_pos = motion_get_current_pos();
  comm_print("ready X%.3f Y%.3f Z%.3f", (double)current_pos.x,
             (double)current_pos.y, (double)current_pos.z);
}

int main() {
  // init core
  state_machine_init();
  comm_init();

  // init hardware
  motor_init();

  // init modules
  motion_init();

  // apply default settings
  settings_apply_all();
  comm_print("Default settings applied");

  // main command processing loop
  comm_print("Spark corefw: Type 'help' for commands");

  while (1) {
    // Wait for command from input thread
    char command[256];
    comm_get_next_command(command);

    // Execute the command
    handle_console_command(command);
  }

  return 0;
}
