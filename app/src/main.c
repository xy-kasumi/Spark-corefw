#include <drivers/tmc_driver.h>

#include "comm.h"
#include "system.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>

// Step generation management
static const struct device* step_gen_cnt =
    DEVICE_DT_GET(DT_NODELABEL(step_gen_cnt));

// Motor device
static const struct device* motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));

static volatile int remaining_steps = 0;  // Positive=forward, negative=backward
static bool current_direction = false;    // false=backward, true=forward
typedef enum {
  STEP_IDLE,        // No stepping in progress
  STEP_PULSE_HIGH,  // Step pin is HIGH (1 tick)
  STEP_PULSE_LOW,   // Step pin is LOW, waiting before next step (1 tick)
} step_state_t;
static step_state_t step_state = STEP_IDLE;

// Handle step pulse generation state machine (called every 30us)
static void main_step_tick() {
  switch (step_state) {
    case STEP_IDLE:
      if (remaining_steps != 0) {
        // Start new step: set direction and begin pulse
        bool dir = (remaining_steps > 0);
        if (dir != current_direction) {
          current_direction = dir;
          tmc_set_dir(motor0, dir);
        }

        // Start step pulse (HIGH)
        tmc_set_step(motor0, true);
        step_state = STEP_PULSE_HIGH;
      }
      break;

    case STEP_PULSE_HIGH:
      // End step pulse (LOW)
      tmc_set_step(motor0, false);
      step_state = STEP_PULSE_LOW;

      // Consume one step
      if (remaining_steps > 0) {
        remaining_steps--;
      } else {
        remaining_steps++;
      }
      break;

    case STEP_PULSE_LOW:
      // Wait one tick before allowing next step
      step_state = STEP_IDLE;
      break;
  }
}

// Step generation ISR handler: manages step pulses (called every 30us)
static void main_step_tick_handler(const struct device* dev, void* user_data) {
  main_step_tick();
}

// Queue a single step (true=forward, false=backward)
void queue_step(bool dir) {
  if (dir) {
    remaining_steps++;
  } else {
    remaining_steps--;
  }
}

// Destructive string parsing helper
// Splits string at first delimiter, null-terminates first part
// Returns pointer to rest of string (after delimiter) or NULL
static char* split_at(char* str, char delim) {
  if (!str)
    return NULL;

  // Skip leading delimiters
  while (*str == delim)
    str++;
  if (*str == '\0')
    return NULL;

  // Find delimiter
  char* p = strchr(str, delim);
  if (p) {
    *p = '\0';  // Terminate first part
    p++;
    // Skip delimiters after split
    while (*p == delim)
      p++;
    return (*p == '\0') ? NULL : p;
  }

  return NULL;  // No delimiter found
}

// Command: help
static void cmd_help(char* args) {
  comm_print("help - Show this help");
  comm_print("regs - Read TMC registers");
  comm_print("steptest - Step motor test");
  comm_print("set <var> <val> - Set variable to value");
  comm_print("! - Cancel current operation");
}

// Command: regs
static void cmd_regs(char* args) {
  char buf[256];
  int ret = tmc_dump_regs(motor0, buf, sizeof(buf));
  if (ret < 0) {
    comm_print_err("Failed to dump registers: %d", ret);
  } else {
    comm_print("%s", buf);
  }
}

// Command: steptest
static void cmd_steptest(char* args) {
  tmc_energize(motor0, true);

  for (int i = 0; i < 2 * 200 * 32; i++) {  // 2 rotations at 32 microsteps
    // Check for cancel request
    if (g_cancel_requested) {
      comm_print_info("Steptest cancelled at step %d", i);
      break;
    }

    queue_step(true);
    k_sleep(K_USEC(250));

    // Print SG_RESULT every 100 steps (50ms intervals at 500us/step)
    if (i % 100 == 0) {
      int sg_result = tmc_sgresult(motor0);
      comm_print("SG:%d", sg_result);
    }

    if (tmc_stalled(motor0)) {
      comm_print("Stall detected at step %d", i);
      break;
    }
  }

  tmc_energize(motor0, false);
}

// Command: set <var> <val>
static void cmd_set(char* args) {
  if (!args) {
    comm_print_err("Usage: set <var> <val>");
    return;
  }

  // Destructive parse: split var and val
  char* var = args;
  char* val = split_at(args, ' ');

  if (!val) {
    comm_print_err("Usage: set <var> <val>");
    return;
  }

  // Handle different variables
  int ret = 0;
  if (strcmp(var, "mot0.microstep") == 0) {
    ret = tmc_set_microstep(motor0, atoi(val));
    if (ret == 0)
      comm_print("Microstep set to %s", val);
  } else if (strcmp(var, "mot0.current") == 0) {
    ret = tmc_set_current(motor0, atoi(val), 0);
    if (ret == 0)
      comm_print("Current set to %s%%", val);
  } else if (strcmp(var, "mot0.thresh") == 0) {
    ret = tmc_set_stallguard_threshold(motor0, atoi(val));
    if (ret == 0)
      comm_print("StallGuard threshold set to %s", val);
  } else {
    comm_print("unknown variable %s", var);
    return;
  }

  if (ret < 0) {
    comm_print_err("Failed to set %s: error %d", var, ret);
  }
}

static void handle_console_command(char* command) {
  g_machine_state = STATE_EXEC_INTERACTIVE;
  comm_print_ack();

  // Destructive parse: split command and arguments
  char* cmd = command;
  char* args = split_at(command, ' ');

  // Dispatch to command handler
  if (strcmp(cmd, "help") == 0) {
    cmd_help(args);
  } else if (strcmp(cmd, "regs") == 0) {
    cmd_regs(args);
  } else if (strcmp(cmd, "steptest") == 0) {
    cmd_steptest(args);
  } else if (strcmp(cmd, "set") == 0) {
    cmd_set(args);
  } else {
    comm_print_err("unknown command: %s; type 'help' for available commands",
                   cmd);
  }

  // Clear cancel flag and return to IDLE
  g_cancel_requested = false;
  g_machine_state = STATE_IDLE;
  comm_print("ready");
}

int main() {
  // init core
  state_machine_init();
  comm_init();

  // init peripherals
  // Check motor device readiness
  if (!device_is_ready(motor0)) {
    comm_print_err("Motor device not ready");
    return -ENODEV;
  }
  comm_print("TMC2209 initialized via device model");

  // Initialize step generation counter
  struct counter_top_cfg step_top_cfg = {
      .callback = main_step_tick_handler,
      .ticks = counter_us_to_ticks(step_gen_cnt,
                                   30),  // 30us ISR -> step pulse generation
  };

  counter_start(step_gen_cnt);
  int ret = counter_set_top_value(step_gen_cnt, &step_top_cfg);
  if (ret < 0) {
    comm_print_err("Step generation timer init failed: %d", ret);
    return ret;
  }
  comm_print("Step generation initialized");

  // apply default settings
  ret = tmc_set_microstep(motor0, 32);
  if (ret < 0) {
    comm_print_err("Failed to set microstep: %d", ret);
  } else {
    comm_print("Microstep set to 32");
  }

  ret = tmc_set_current(motor0, 30, 0);
  if (ret < 0) {
    comm_print_err("Failed to set current: %d", ret);
  } else {
    comm_print("Current set: run=30%% hold=0%%");
  }

  ret = tmc_set_stallguard_threshold(motor0, 2);
  if (ret < 0) {
    comm_print_err("Failed to set stallguard threshold: %d", ret);
  } else {
    comm_print("StallGuard threshold set to 2");
  }

  ret = tmc_set_tcoolthrs(motor0,
                          750000);  // make this bigger to make stallguard work
                                    // at lower speed (might be noisier)
  if (ret < 0) {
    comm_print_err("Failed to set TCOOLTHRS: %d", ret);
  } else {
    comm_print("TCOOLTHRS set to 750000");
  }

  // main command processing loop
  comm_print("Spark corefw: Type 'help' for commands");

  while (1) {
    // Wait for command from input thread
    char command[256];
    comm_get_next_command(command);

    // Execute the command
    handle_console_command(command);
  }

  return 0;
}
