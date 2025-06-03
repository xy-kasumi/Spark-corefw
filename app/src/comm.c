#include "comm.h"

#include "comm_raw.h"
#include "system.h"

#include <stdarg.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

// Static buffer for command reading
static char command_buffer[256];

void comm_init() {
  comm_raw_init();
  state_machine_init();
}

void comm_print(const char* fmt, ...) {
  machine_state_t state = state_machine_get_state();

  // Add prefix based on state
  switch (state) {
    case STATE_IDLE:
      comm_raw_puts("I ");
      break;
    case STATE_EXEC_INTERACTIVE:
      comm_raw_puts(">");
      break;
    case STATE_EXEC_STREAM:
      comm_raw_puts("@");
      break;
  }

  // Output the message
  va_list args;
  va_start(args, fmt);
  comm_raw_vprintf(fmt, args);
  va_end(args);
  
  // Auto-add newline
  comm_raw_puts("\n");
}

void comm_print_ack() {
  comm_raw_puts(">ack\n");
}

void comm_print_err(const char* fmt, ...) {
  machine_state_t state = state_machine_get_state();

  // Choose prefix based on state
  if (state == STATE_EXEC_INTERACTIVE) {
    comm_raw_puts(">err ");
  } else if (state == STATE_EXEC_STREAM) {
    comm_raw_puts("@err ");
  } else {
    // In IDLE, errors still use "I " prefix
    comm_raw_puts("I ");
  }

  va_list args;
  va_start(args, fmt);
  comm_raw_vprintf(fmt, args);
  va_end(args);
  
  // Auto-add newline
  comm_raw_puts("\n");
}

void comm_print_info(const char* fmt, ...) {
  machine_state_t state = state_machine_get_state();

  // Choose prefix based on state
  if (state == STATE_EXEC_INTERACTIVE) {
    comm_raw_puts(">inf ");
  } else if (state == STATE_EXEC_STREAM) {
    comm_raw_puts("@inf ");
  } else {
    // In IDLE, info uses "I " prefix
    comm_raw_puts("I ");
  }

  va_list args;
  va_start(args, fmt);
  comm_raw_vprintf(fmt, args);
  va_end(args);
  
  // Auto-add newline
  comm_raw_puts("\n");
}

char* comm_read_command() {
  static int buffer_pos = 0;
  unsigned char ch;

  // Get the console UART device for polling input
  const struct device* console_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
  if (!device_is_ready(console_uart_dev)) {
    comm_print_err("Console UART not ready");
    return NULL;
  }

  while (1) {
    // Poll for incoming characters
    if (uart_poll_in(console_uart_dev, &ch) == 0) {
      // Character received

      // Handle special characters
      if (ch == '\r' || ch == '\n') {
        // End of line - process the command
        if (buffer_pos > 0) {
          command_buffer[buffer_pos] = '\0';  // Null terminate
          
          // Trim leading whitespace
          char* trimmed = command_buffer;
          while (*trimmed == ' ' || *trimmed == '\t') {
            trimmed++;
          }
          
          buffer_pos = 0;  // Reset buffer for next command
          
          // Return command if not empty
          if (strlen(trimmed) > 0) {
            return trimmed;
          }
        }
      } else if (ch == '\b' || ch == 0x7F) {
        // Backspace or DEL - remove last character
        if (buffer_pos > 0) {
          buffer_pos--;
          comm_raw_puts("\b \b");  // Echo backspace
        }
      } else if (ch >= 0x20 && ch <= 0x7E) {
        // Printable ASCII character
        if (buffer_pos < sizeof(command_buffer) - 1) {
          command_buffer[buffer_pos++] = ch;
          // No echo to avoid double output
        }
      }
      // Ignore other control characters
    } else {
      // No character available, sleep briefly to avoid busy waiting
      k_sleep(K_MSEC(1));
    }
  }
}