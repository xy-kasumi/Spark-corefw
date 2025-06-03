#include "comm.h"
#include "tmc2209.h"

#include <stdlib.h>
#include <string.h>

// Handle set command: "set var val"
static void handle_set(const char* var, const char* val) {
  if (strcmp(var, "mot0.microstep") == 0) {
    tmc_set_microstep(atoi(val));
  } else if (strcmp(var, "mot0.curr") == 0) {
    tmc_set_current(atoi(val), 0);
  } else {
    comm_print("unknown variable %s", var);
  }
}

static void handle_console_command(const char* command) {
  comm_print("Received command: %s", command);

  // Add your custom command handlers here
  if (strcmp(command, "help") == 0) {
    comm_print("help - Show this help");
    comm_print("regs - Read TMC registers");
    comm_print("step <count> - Step motor <count> times");
    comm_print("set <var> <val> - Set variable to value");
  } else if (strcmp(command, "regs") == 0) {
    tmc_dump_regs();
  } else if (strncmp(command, "step ", 5) == 0) {
    int steps = atoi(command + 5);
    comm_print("Stepping %d times", steps);
  } else if (strncmp(command, "set ", 4) == 0) {
    // Parse "set var val"
    const char* rest = command + 4;  // Skip "set "
    char* space = strchr(rest, ' ');
    if (space != NULL) {
      *space = '\0';  // Temporarily null-terminate var
      const char* var = rest;
      const char* val = space + 1;
      handle_set(var, val);
      *space = ' ';  // Restore original string
    } else {
      comm_print_err("Usage: set <var> <val>");
    }
  } else {
    comm_print_err("Unknown command: %s", command);
    comm_print("Type 'help' for available commands");
  }
}

int main() {
  comm_init();

  // initialize hardware
  tmc_init();

  // main command processing loop
  comm_print("Spark corefw: Type 'help' for commands");
  while (1) {
    char* command = comm_read_command();
    if (command) {
      handle_console_command(command);
    }
  }

  return 0;
}
