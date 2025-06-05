// Application entry point for spark core firmware.
// Main command loop is executed here.
#include "comm.h"
#include "gcode.h"
#include "settings.h"
#include "strutil.h"
#include "system.h"

#include <drivers/tmc_driver.h>

#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>

// Hardware devices
static const struct device* step_gen_cnt =
    DEVICE_DT_GET(DT_NODELABEL(step_gen_cnt));
const struct device* motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));
const struct device* motor1 = DEVICE_DT_GET(DT_NODELABEL(motor1));
const struct device* motor2 = DEVICE_DT_GET(DT_NODELABEL(motor2));

// Step generation state
static volatile int remaining_steps = 0;  // Positive=forward, negative=backward
static bool current_direction = false;    // false=backward, true=forward

// Step pulse generation state machine
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

// Command: help
static void cmd_help(char* args) {
  comm_print("help - Show this help");
  comm_print("regs - Read TMC registers");
  comm_print("steptest - Step motor test");
  comm_print("set <var> <val> - Set variable to value");
  comm_print("get - List all variables with values");
  comm_print("get <var> - Get specific variable value");
  comm_print("! - Cancel current operation");
}

// Command: regs
static void cmd_regs(char* args) {
  char buf[256];
  const struct device* motors[] = {motor0, motor1, motor2};
  const char* names[] = {"mot0", "mot1", "mot2"};

  for (int i = 0; i < 3; i++) {
    int ret = tmc_dump_regs(motors[i], buf, sizeof(buf));
    if (ret < 0) {
      comm_print("%s: error %d", names[i], ret);
    } else {
      comm_print("%s: %s", names[i], buf);
    }
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

// Command: gcode
static void cmd_gcode(char* full_command) {
  gcode_parsed_t parsed;
  if (parse_gcode(full_command, &parsed)) {
    comm_print("G%d parsed: X=%s Y=%s Z=%s", parsed.command,
               parsed.has_x ? "yes" : "no", parsed.has_y ? "yes" : "no",
               parsed.has_z ? "yes" : "no");
  } else {
    comm_print_err("Failed to parse G-code: %s", full_command);
  }
}

// Command: set <var> <val>
static void cmd_set(char* args) {
  if (!args) {
    comm_print_err("Usage: set <var> <val>");
    return;
  }

  // Destructive parse: split var and val
  char* parse_ptr = args;
  char* var = split_front(&parse_ptr, ' ');
  char* val = parse_ptr;

  if (!val) {
    comm_print_err("Usage: set <var> <val>");
    return;
  }

  if (!settings_set(var, val)) {
    comm_print_err("Failed to set %s", var);
  }
}

// Command: get [var]
static void cmd_get(char* args) {
  if (!args || strlen(args) == 0) {
    // List all settings
    const char* key;
    float value;
    for (int i = 0; settings_get_by_index(i, &key, &value); i++) {
      comm_print("%s %.1f", key, (double)value);
    }
  } else {
    // Get specific setting - check if key exists
    bool key_exists = false;
    const char* test_key;
    for (int i = 0; settings_get_by_index(i, &test_key, NULL); i++) {
      if (strcmp(test_key, args) == 0) {
        key_exists = true;
        break;
      }
    }
    
    if (key_exists) {
      float value = settings_get(args);
      comm_print("%.1f", (double)value);
    } else {
      comm_print_err("Unknown variable %s", args);
    }
  }
}

static void handle_console_command(char* command) {
  g_machine_state = STATE_EXEC_INTERACTIVE;
  comm_print_ack();

  // Check for G-code before destructive parsing
  if (command[0] == 'G') {
    cmd_gcode(command);  // Pass full command for G-code parsing
    goto cleanup;
  }

  // Destructive parse: split command and arguments
  char* parse_ptr = command;
  char* cmd = split_front(&parse_ptr, ' ');
  char* args = parse_ptr;  // Remaining string after command

  // Dispatch to command handler
  if (strcmp(cmd, "help") == 0) {
    cmd_help(args);
  } else if (strcmp(cmd, "regs") == 0) {
    cmd_regs(args);
  } else if (strcmp(cmd, "steptest") == 0) {
    cmd_steptest(args);
  } else if (strcmp(cmd, "set") == 0) {
    cmd_set(args);
  } else if (strcmp(cmd, "get") == 0) {
    cmd_get(args);
  } else {
    comm_print_err("unknown command: %s; type 'help' for available commands",
                   cmd);
  }

cleanup:
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
  if (!device_is_ready(motor0)) {
    comm_print_err("Motor0 device not ready");
    return -ENODEV;
  }
  if (!device_is_ready(motor1)) {
    comm_print_err("Motor1 device not ready");
    return -ENODEV;
  }
  if (!device_is_ready(motor2)) {
    comm_print_err("Motor2 device not ready");
    return -ENODEV;
  }

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
  settings_apply_all();
  comm_print("Default settings applied");

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
