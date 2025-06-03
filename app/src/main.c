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

static const struct gpio_dt_spec step0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(step0), gpios);

static const struct gpio_dt_spec en0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(en0), gpios);

static const struct gpio_dt_spec dir0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dir0), gpios);

static const struct gpio_dt_spec muart0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(muart0), gpios);

// Handle set command: "set var val"
void handle_set(const char* var, const char* val) {
  if (strcmp(var, "mot0.curr") == 0) {
    // TODO: do bunch of TMC calls.

  } else {
    comm_print("unknown variable %s\n", var);
  }
}

// Handle a complete line received from serial console
void handle_console_line(const char* line) {
  // Trim whitespace
  while (*line == ' ' || *line == '\t')
    line++;

  if (strlen(line) == 0) {
    return;  // Empty line
  }

  comm_print("Received command: %s\n", line);

  // Add your custom command handlers here
  if (strcmp(line, "help") == 0) {
    comm_print("help - Show this help\n");
    comm_print("regs - Read TMC registers\n");
    comm_print("step <count> - Step motor <count> times\n");
    comm_print("set <var> <val> - Set variable to value\n");
  } else if (strcmp(line, "regs") == 0) {
    dump_tmc_regs();
  } else if (strncmp(line, "step ", 5) == 0) {
    int steps = atoi(line + 5);
    comm_print("Stepping %d times\n", steps);
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
      comm_print_err("Usage: set <var> <val>\n");
    }
  } else {
    comm_print_err("Unknown command: %s\n", line);
    comm_print("Type 'help' for available commands\n");
  }
}

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

  comm_print("Spark corefw: Type 'help' for commands\n");

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
          // console_puts("\b \b");
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
  if (!gpio_is_ready_dt(&step0)) {
    comm_print_err("LED GPIO not ready\n");
    return 0;
  }

  int ret = gpio_pin_configure_dt(&step0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    comm_print_err("Could not configure step0 GPIO (%d)\n", ret);
    return 0;
  }

  ret = gpio_pin_configure_dt(&dir0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    comm_print_err("Could not configure dir0 GPIO (%d)\n", ret);
    return 0;
  }

  ret = gpio_pin_configure_dt(&en0,
                              GPIO_OUTPUT_ACTIVE);  // ACTIVE == 1 == disabled
  if (ret < 0) {
    comm_print_err("Could not configure en0 GPIO (%d)\n", ret);
    return 0;
  }

  // active is idle
  // for some reason, GPIO_OPEN_DRAIN must be configured now (instead of writing
  // time). Otherwise changing to GPIO_INPUT and reading didn't work.
  ret = gpio_pin_configure_dt(&muart0, GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);
  if (ret < 0) {
    comm_print_err("Could not configure muart0 GPIO (%d)\n", ret);
    return 0;
  }

  tmc_init();
  comm_print("Starting main loop\n");
  k_sleep(K_FOREVER);

  while (true) {
    dump_tmc_regs();
    k_sleep(K_MSEC(5000));
  }

  while (1) {
    gpio_pin_set_dt(&step0, true);
    k_sleep(K_USEC(10));
    gpio_pin_set_dt(&step0, false);

    k_sleep(K_MSEC(1));
  }

  return 0;
}
