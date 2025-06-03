#include "comm.h"

#include "comm_raw.h"
#include "system.h"

#include <stdarg.h>

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