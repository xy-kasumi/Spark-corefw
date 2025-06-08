// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "system.h"

// Global state variable
machine_state_t g_machine_state = STATE_IDLE;

// Cancel request flag
volatile bool g_cancel_requested = false;

void state_machine_init() {
  g_machine_state = STATE_IDLE;
  g_cancel_requested = false;
}

machine_state_t state_machine_get_state() {
  return g_machine_state;
}

const char* state_machine_get_state_name(machine_state_t state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_EXEC_INTERACTIVE:
      return "EXEC_INTERACTIVE";
    case STATE_EXEC_STREAM:
      return "EXEC_STREAM";
    default:
      return "UNKNOWN";
  }
}
