// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "comm_raw.h"

#include <stdarg.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

// Global console UART device
static const struct device* console_uart_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

void comm_raw_init() {
  if (!device_is_ready(console_uart_dev)) {
    // not ready, but no way to tell user.
  }
}

void comm_raw_putc(char c) {
  if (c == '\n') {
    uart_poll_out(console_uart_dev, '\r');
  }
  uart_poll_out(console_uart_dev, c);
}

void comm_raw_puts(const char* str) {
  while (*str) {
    comm_raw_putc(*str++);
  }
}

void comm_raw_printf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  comm_raw_vprintf(fmt, args);
  va_end(args);
}

void comm_raw_vprintf(const char* fmt, va_list args) {
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  comm_raw_puts(buffer);
}

int comm_raw_poll_in(unsigned char* ch) {
  return uart_poll_in(console_uart_dev, ch);
}
