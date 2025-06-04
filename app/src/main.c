#include "comm.h"
#include "system.h"
#include "tmc2209.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

// Destructive string parsing helper
// Splits string at first delimiter, null-terminates first part
// Returns pointer to rest of string (after delimiter) or NULL
static char* split_at(char* str, char delim) {
  if (!str)
    return NULL;

  // Skip leading delimiters
  while (*str == delim)
    str++;
  if (*str == '\0')
    return NULL;

  // Find delimiter
  char* p = strchr(str, delim);
  if (p) {
    *p = '\0';  // Terminate first part
    p++;
    // Skip delimiters after split
    while (*p == delim)
      p++;
    return (*p == '\0') ? NULL : p;
  }

  return NULL;  // No delimiter found
}

// Command: help
static void cmd_help(char* args) {
  comm_print("help - Show this help");
  comm_print("regs - Read TMC registers");
  comm_print("steptest - Step motor test");
  comm_print("set <var> <val> - Set variable to value");
  comm_print("! - Cancel current operation");
}

// Command: regs
static void cmd_regs(char* args) {
  tmc_dump_regs();
}

// Command: steptest
static void cmd_steptest(char* args) {
  tmc_energize(true);

  for (int i = 0; i < 2 * 200 * 32; i++) {  // 2 rotations at 32 microsteps
    // Check for cancel request
    if (g_cancel_requested) {
      comm_print_info("Steptest cancelled at step %d", i);
      break;
    }

    tmc_step(true);
    k_sleep(K_USEC(250));

    // Print SG_RESULT every 100 steps (50ms intervals at 500us/step)
    if (i % 100 == 0) {
      int sg_result = tmc_sgresult();
      comm_print("SG:%d", sg_result);
    }

    if (tmc_stalled()) {
      comm_print("Stall detected at step %d", i);
      break;
    }
  }

  tmc_energize(false);
}

// Command: set <var> <val>
static void cmd_set(char* args) {
  if (!args) {
    comm_print_err("Usage: set <var> <val>");
    return;
  }

  // Destructive parse: split var and val
  char* var = args;
  char* val = split_at(args, ' ');

  if (!val) {
    comm_print_err("Usage: set <var> <val>");
    return;
  }

  // Handle different variables
  if (strcmp(var, "mot0.microstep") == 0) {
    tmc_set_microstep(atoi(val));
  } else if (strcmp(var, "mot0.current") == 0) {
    tmc_set_current(atoi(val), 0);
  } else if (strcmp(var, "mot0.thresh") == 0) {
    tmc_set_stallguard_threshold(atoi(val));
  } else {
    comm_print("unknown variable %s", var);
  }
}

static void handle_console_command(char* command) {
  g_machine_state = STATE_EXEC_INTERACTIVE;
  comm_print_ack();

  // Destructive parse: split command and arguments
  char* cmd = command;
  char* args = split_at(command, ' ');

  // Dispatch to command handler
  if (strcmp(cmd, "help") == 0) {
    cmd_help(args);
  } else if (strcmp(cmd, "regs") == 0) {
    cmd_regs(args);
  } else if (strcmp(cmd, "steptest") == 0) {
    cmd_steptest(args);
  } else if (strcmp(cmd, "set") == 0) {
    cmd_set(args);
  } else {
    comm_print_err("unknown command: %s; type 'help' for available commands",
                   cmd);
  }

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
  tmc_init();

  // apply default settings
  tmc_set_microstep(32);
  tmc_set_current(30, 0);
  tmc_set_stallguard_threshold(2);
  tmc_set_tcoolthrs(750000);  // make this bigger to make stallguard work at
                              // lower speed (might be noisier)

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
