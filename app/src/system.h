/**
 * (Singleton) State machine which entire app depends on.
 */
#pragma once

#include <stdbool.h>

/** Machine states according to the design doc */
typedef enum {
  STATE_IDLE,  // Machine is not executing command, ready to accept commands
  STATE_EXEC_INTERACTIVE,  // Machine is executing a single isolated command
  STATE_EXEC_STREAM  // Machine is executing a sequence of streamed commands
                     // (future)
} machine_state_t;

/** Global state (will be properly encapsulated later) */
extern machine_state_t g_machine_state;

/** Cancel request flag (volatile for thread safety) */
extern volatile bool g_cancel_requested;

// State machine functions

void state_machine_init();
machine_state_t state_machine_get_state();
const char* state_machine_get_state_name(machine_state_t state);
