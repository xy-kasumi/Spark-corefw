#include "comm.h"

#include "comm_raw.h"
#include "system.h"

#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>

// Static buffer for command reading
static char command_buffer[256];

void comm_init() {
  comm_raw_init();
}

void comm_print(const char* fmt, ...) {
  switch (state_machine_get_state()) {
    case STATE_IDLE:
      comm_raw_puts("I ");
      break;
    case STATE_EXEC_INTERACTIVE:
      comm_raw_puts("> ");
      break;
    case STATE_EXEC_STREAM:
      comm_raw_puts("@ ");
      break;
  }

  va_list args;
  va_start(args, fmt);
  comm_raw_vprintf(fmt, args);
  va_end(args);

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

  while (1) {
    unsigned char ch;
    if (comm_raw_poll_in(&ch) != 0) {
      k_sleep(K_MSEC(1));
      continue;
    }

    if (ch == '\r' || ch == '\n') {
      // end of line
      comm_raw_putc('\n');

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
        // overwrite with space (assuming local echo already moved cursor back)
        comm_raw_puts(" \b");
        buffer_pos--;
      }
    } else if (ch >= 0x20 && ch <= 0x7E) {
      // Printable ASCII character
      if (buffer_pos < sizeof(command_buffer) - 1) {
        command_buffer[buffer_pos] = ch;
        buffer_pos++;
      }
    }
  }
}