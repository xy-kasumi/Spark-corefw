#include "console_utils.h"
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
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

void dump_tmc_regs() {
  uint32_t res;

  res = tmc_tx_regread(0x00);
  console_printf("GCONF: 0x%08x\n", res);
  k_sleep(K_MSEC(10));

  res = tmc_tx_regread(0x06);
  console_printf("IOIN: 0x%08x\n", res);
  k_sleep(K_MSEC(10));

  res = tmc_tx_regread(0x41);
  console_printf("SG_RESULT: 0x%08x\n", res);
  k_sleep(K_MSEC(10));

  res = tmc_tx_regread(0x6c);
  console_printf("CHOPCONF: 0x%08x\n", res);
  k_sleep(K_MSEC(10));
}

static const struct gpio_dt_spec step0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(step0), gpios);

static const struct gpio_dt_spec en0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(en0), gpios);

static const struct gpio_dt_spec dir0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dir0), gpios);

static const struct gpio_dt_spec muart0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(muart0), gpios);

// Handle a complete line received from serial console
void handle_console_line(const char* line) {
  // Trim whitespace
  while (*line == ' ' || *line == '\t')
    line++;

  if (strlen(line) == 0) {
    return;  // Empty line
  }

  console_printf("Received command: %s\n", line);

  // Add your custom command handlers here
  if (strcmp(line, "help") == 0) {
    console_puts("I Available commands:\n");
    console_puts("I help - Show this help\n");
    console_puts("I regs - Read TMC registers\n");
    console_puts("I step <count> - Step motor <count> times\n");
  } else if (strcmp(line, "regs") == 0) {
    dump_tmc_regs();
  } else if (strncmp(line, "step ", 5) == 0) {
    int steps = atoi(line + 5);
    console_printf("I Stepping %d times\n", steps);
  } else {
    console_printf("I Unknown command: %s\n", line);
    console_puts("I Type 'help' for available commands\n");
  }
}

void console_thread() {
  char line_buffer[256];
  int buffer_pos = 0;
  unsigned char ch;

  // Initialize console utilities
  console_utils_init();

  // Get the console UART device for polling input
  const struct device* console_uart_dev =
      DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
  if (!device_is_ready(console_uart_dev)) {
    printk("Console UART device not ready\n");
    return;
  }

  console_puts("\n=== Spark Console ===\n");
  console_puts("Type 'help' for available commands\n");

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
    LOG_ERR("LED GPIO not ready");
    return 0;
  }

  int ret = gpio_pin_configure_dt(&step0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    LOG_ERR("Could not configure step0 GPIO (%d)", ret);
    return 0;
  }

  ret = gpio_pin_configure_dt(&dir0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    LOG_ERR("Could not configure dir0 GPIO (%d)", ret);
    return 0;
  }

  ret = gpio_pin_configure_dt(&en0,
                              GPIO_OUTPUT_ACTIVE);  // ACTIVE == 1 == disabled
  if (ret < 0) {
    LOG_ERR("Could not configure en0 GPIO (%d)", ret);
    return 0;
  }

  // active is idle
  // for some reason, GPIO_OPEN_DRAIN must be configured now (instead of writing
  // time). Otherwise changing to GPIO_INPUT and reading didn't work.
  ret = gpio_pin_configure_dt(&muart0, GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);
  if (ret < 0) {
    LOG_ERR("Could not configure muart0 GPIO (%d)", ret);
    return 0;
  }

  tmc_init();
  printk("Starting main loop\n");
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
