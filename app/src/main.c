#include "comm.h"
#include "tmc2209.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/console/console.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>


// Handle set command: "set var val"
void handle_set(const char* var, const char* val) {
  if (strcmp(var, "mot0.microstep") == 0) {
    tmc_set_microstep(atoi(val));
  } else if (strcmp(var, "mot0.curr") == 0) {
    tmc_set_current(atoi(val), 0);
  } else {
    comm_print("unknown variable %s", var);
  }
}

// Handle a complete line received from serial console
// TODO: Move line cleanup to comm.c, and only expose cleaned line to handler.
void handle_console_line(const char* line) {
  // Trim whitespace
  while (*line == ' ' || *line == '\t')
    line++;

  if (strlen(line) == 0) {
    return;  // Empty line
  }

  comm_print("Received command: %s", line);

  // Add your custom command handlers here
  if (strcmp(line, "help") == 0) {
    comm_print("help - Show this help");
    comm_print("regs - Read TMC registers");
    comm_print("step <count> - Step motor <count> times");
    comm_print("set <var> <val> - Set variable to value");
  } else if (strcmp(line, "regs") == 0) {
    tmc_dump_regs();
  } else if (strncmp(line, "step ", 5) == 0) {
    int steps = atoi(line + 5);
    comm_print("Stepping %d times", steps);
  } else if (strncmp(line, "set ", 4) == 0) {
    // Parse "set var val"
    const char* rest = line + 4;  // Skip "set "
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
    comm_print_err("Unknown command: %s", line);
    comm_print("Type 'help' for available commands");
  }
}

// TODO: Move to comm.h / comm.c
void console_thread() {
  char line_buffer[256];
  int buffer_pos = 0;
  unsigned char ch;

  // Get the console UART device for polling input
  const struct device* console_uart_dev =
      DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
  if (!device_is_ready(console_uart_dev)) {
    // Can't use console functions before init, just return silently
    return;
  }

  // Initialize communication subsystem
  comm_init();

  comm_print("Spark corefw: Type 'help' for commands");

  while (1) {
    // Poll for incoming characters
    if (uart_poll_in(console_uart_dev, &ch) == 0) {
      // Character received

      // Handle special characters
      if (ch == '\r' || ch == '\n') {
        // End of line - process the command
        if (buffer_pos > 0) {
          line_buffer[buffer_pos] = '\0';  // Null terminate
          handle_console_line(line_buffer);
          buffer_pos = 0;  // Reset buffer
        }
      } else if (ch == '\b' || ch == 0x7F) {
        // Backspace or DEL - remove last character
        if (buffer_pos > 0) {
          buffer_pos--;
          // Optionally send backspace sequence to terminal
          // comm_raw_puts("\b \b");
        }
      } else if (ch >= 0x20 && ch <= 0x7E) {
        // Printable ASCII character
        if (buffer_pos < sizeof(line_buffer) - 1) {
          line_buffer[buffer_pos++] = ch;
          // No echo - this eliminates the echo behavior
        }
      }
      // Ignore other control characters
    } else {
      // No character available, sleep briefly to avoid busy waiting
      k_sleep(K_MSEC(1));
    }
  }
}

K_THREAD_DEFINE(console_tid, 1024, console_thread, NULL, NULL, NULL, 5, 0, 0);

int main() {
  tmc_init();
  comm_print("Starting main loop");
  k_sleep(K_FOREVER);
  return 0;
}
