// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "comm.h"

#include "comm_raw.h"
#include "system.h"

#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>

// Static buffer for command reading
static char command_buffer[256];

// Command message for thread communication
typedef struct {
  char cmd[256];
} cmd_msg_t;

// Single-entry message queue for commands
K_MSGQ_DEFINE(cmd_queue, sizeof(cmd_msg_t), 1, 4);

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

void comm_get_next_command(char* buffer) {
  cmd_msg_t msg;
  k_msgq_get(&cmd_queue, &msg, K_FOREVER);
  strcpy(buffer, msg.cmd);
}

// Input thread function
static void input_thread_fn(void* p1, void* p2, void* p3) {
  while (1) {
    char* cmd = comm_read_command();

    // Special handling for "!" - always process immediately
    if (strcmp(cmd, "!") == 0) {
      g_cancel_requested = true;
      continue;
    }

    // In EXEC_INTERACTIVE, silently reject other commands
    if (g_machine_state != STATE_IDLE) {
      continue;
    }

    // Queue command for main thread
    cmd_msg_t msg;
    strncpy(msg.cmd, cmd, sizeof(msg.cmd) - 1);
    msg.cmd[sizeof(msg.cmd) - 1] = '\0';

    k_msgq_put(&cmd_queue, &msg, K_FOREVER);
  }
}

// Define input thread with higher priority than main
K_THREAD_DEFINE(input_thread,
                2048,
                input_thread_fn,
                NULL,
                NULL,
                NULL,
                5,  // Higher priority than main (7)
                0,
                0);

// Adler-32 checksum calculation
static uint32_t adler32(uint8_t* data, int len) {
  uint32_t a = 1, b = 0;
  for (int i = 0; i < len; i++) {
    a = (a + data[i]) % 65521;
    b = (b + a) % 65521;
  }
  return (b << 16) | a;
}

void comm_print_blob(uint8_t* ptr, int size) {
  const char* base64url_table =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  const char* hex_table = "0123456789abcdef";

  comm_raw_puts(">blob ");

  // Encode in base64url
  for (int i = 0; i < size; i += 3) {
    uint32_t val = 0;
    int chars = 0;

    // Pack up to 3 bytes into 24-bit value
    for (int j = 0; j < 3 && i + j < size; j++) {
      val = (val << 8) | ptr[i + j];
      chars++;
    }

    // Pad with zeros if needed
    val <<= (3 - chars) * 8;

    // Output base64url characters (no padding)
    char b64_chars[4];
    b64_chars[0] = base64url_table[(val >> 18) & 0x3F];
    b64_chars[1] = base64url_table[(val >> 12) & 0x3F];
    b64_chars[2] = base64url_table[(val >> 6) & 0x3F];
    b64_chars[3] = base64url_table[val & 0x3F];

    int output_chars = (chars == 1) ? 2 : (chars == 2) ? 3 : 4;
    for (int k = 0; k < output_chars; k++) {
      comm_raw_putc(b64_chars[k]);
    }
  }

  // Calculate and append Adler-32 checksum
  uint32_t checksum = adler32(ptr, size);
  char checksum_str[9];  // 8 hex chars + null terminator
  checksum_str[0] = hex_table[(checksum >> 28) & 0x0F];
  checksum_str[1] = hex_table[(checksum >> 24) & 0x0F];
  checksum_str[2] = hex_table[(checksum >> 20) & 0x0F];
  checksum_str[3] = hex_table[(checksum >> 16) & 0x0F];
  checksum_str[4] = hex_table[(checksum >> 12) & 0x0F];
  checksum_str[5] = hex_table[(checksum >> 8) & 0x0F];
  checksum_str[6] = hex_table[(checksum >> 4) & 0x0F];
  checksum_str[7] = hex_table[checksum & 0x0F];
  checksum_str[8] = '\0';

  comm_raw_putc(' ');
  comm_raw_puts(checksum_str);
  comm_raw_puts("\n");
}
