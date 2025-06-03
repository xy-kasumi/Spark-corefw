#include "console_utils.h"

#include <stdarg.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

// Global console UART device pointer
static const struct device* console_uart_dev;

void console_utils_init() {
  console_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
}

void console_putc(char c) {
  if (console_uart_dev && device_is_ready(console_uart_dev)) {
    if (c == '\n') {
      uart_poll_out(console_uart_dev, '\r');
    }
    uart_poll_out(console_uart_dev, c);
  }
}

void console_puts(const char* str) {
  while (*str) {
    console_putc(*str++);
  }
}

void console_printf(const char* fmt, ...) {
  char buffer[256];
  va_list args;

  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  console_puts(buffer);
}
