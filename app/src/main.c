// Application entry point for spark core firmware.
// Main command loop is executed here.
#include "comm.h"
#include "gcode.h"
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
  comm_print("regs - Show motor registers");
  comm_print("steptest <motor_num> - Step motor test (0, 1, or 2)");
  comm_print("set <key> <value> - Set variable to value");
  comm_print("get - List all variables with values");
  comm_print("get <key> - Get specific variable value");
  comm_print("! - Cancel current operation");
}

// Command: gcode
static void cmd_gcode(char* full_command) {
  gcode_parsed_t parsed;
  if (parse_gcode(full_command, &parsed)) {
    comm_print("G%d parsed: X=%s Y=%s Z=%s", parsed.command,
               parsed.has_x ? "yes" : "no", parsed.has_y ? "yes" : "no",
               parsed.has_z ? "yes" : "no");
  } else {
    comm_print_err("Failed to parse G-code: %s", full_command);
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

// Command: regs
static void cmd_regs(char* args) {
  motor_dump_registers();
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
  } else if (strcmp(cmd, "regs") == 0) {
    cmd_regs(args);
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
  // Clear cancel flag and return to IDLE
  g_cancel_requested = false;
  g_machine_state = STATE_IDLE;
  comm_print("ready");
}

int main() {
  // init core
  state_machine_init();
  comm_init();

  // init peripherals
  motor_init();

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
