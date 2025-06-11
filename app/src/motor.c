// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "motor.h"
#include "comm.h"
#include "system.h"

#include <drivers/tmc_driver.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>

// Hardware devices
static const struct device* step_gen_cnt =
    DEVICE_DT_GET(DT_NODELABEL(step_gen_cnt));
static const struct device* motor0 = DEVICE_DT_GET(DT_NODELABEL(motor0));
static const struct device* motor1 = DEVICE_DT_GET(DT_NODELABEL(motor1));
static const struct device* motor2 = DEVICE_DT_GET(DT_NODELABEL(motor2));
static const struct device* motor3 = DEVICE_DT_GET(DT_NODELABEL(motor3));
static const struct device* motor4 = DEVICE_DT_GET(DT_NODELABEL(motor4));
static const struct device* motor5 = DEVICE_DT_GET(DT_NODELABEL(motor5));
static const struct device* motor6 = DEVICE_DT_GET(DT_NODELABEL(motor6));

// Step pulse generation state machine
typedef enum {
  STEP_IDLE,        // No stepping in progress
  STEP_PULSE_HIGH,  // Step pin is HIGH (1 tick)
  STEP_PULSE_LOW,   // Step pin is LOW, waiting before next step (1 tick)
} step_state_t;

// Motor idle timeout configuration
static const uint32_t STEP_ISR_PERIOD_US = 30;  // ISR period in microseconds

// Per-motor step generation state
typedef struct {
  const struct device* device;  // Motor device reference

  volatile int target_steps;   // Target position in microsteps
  volatile int current_steps;  // Current position in microsteps

  bool current_direction;   // Current direction state
  step_state_t step_state;  // Current state machine state

  bool always_energized;        // If true, never de-energize due to timeout
  uint32_t idle_timeout_ticks;  // Timeout in ticks (only used if
                                // !always_energized)
  bool energized;               // Current energization state
  uint32_t idle_ticks;          // Ticks since motor became idle
} motor_step_state_t;

static motor_step_state_t motor_states[MOTOR_COUNT];

// Helper to ensure motor energization state
static inline void ensure_energized(motor_step_state_t* motor, bool energize) {
  if (motor->energized != energize) {
    tmc_energize(motor->device, energize);
    motor->energized = energize;
  }
}

// Process step generation for a single motor
static void process_motor_step(motor_step_state_t* motor) {
  switch (motor->step_state) {
    case STEP_IDLE:
      if (motor->current_steps != motor->target_steps) {
        // Need to step toward target - ensure energized
        motor->idle_ticks = 0;
        ensure_energized(motor, true);

        bool dir = (motor->target_steps > motor->current_steps);
        if (dir != motor->current_direction) {
          motor->current_direction = dir;
          tmc_set_dir(motor->device, dir);
        }

        // Start step pulse (HIGH)
        tmc_set_step(motor->device, true);
        motor->step_state = STEP_PULSE_HIGH;
      } else {
        // Motor is at target - handle idle timeout
        if (!motor->always_energized) {
          if (motor->idle_ticks < motor->idle_timeout_ticks) {
            motor->idle_ticks++;
          } else {
            // De-energize after timeout
            ensure_energized(motor, false);
          }
        }
      }
      break;

    case STEP_PULSE_HIGH:
      // End step pulse (LOW)
      tmc_set_step(motor->device, false);
      motor->step_state = STEP_PULSE_LOW;

      // Update current position
      if (motor->target_steps > motor->current_steps) {
        motor->current_steps++;
      } else {
        motor->current_steps--;
      }
      break;

    case STEP_PULSE_LOW:
      // Wait one tick before allowing next step
      motor->step_state = STEP_IDLE;
      break;
  }
}

// Step generation ISR handler: manages step pulses (called every 30us)
static void step_tick_handler(const struct device* dev, void* user_data) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    process_motor_step(&motor_states[i]);
  }
}

void queue_step(int motor_num, bool dir) {
  if (motor_num < 0 || motor_num >= MOTOR_COUNT) {
    return;  // Invalid motor number
  }

  // Atomic increment/decrement of target
  if (dir) {
    __atomic_add_fetch(&motor_states[motor_num].target_steps, 1,
                       __ATOMIC_SEQ_CST);
  } else {
    __atomic_sub_fetch(&motor_states[motor_num].target_steps, 1,
                       __ATOMIC_SEQ_CST);
  }
}

const struct device* motor_get_device(int motor_num) {
  if (motor_num < 0 || motor_num >= MOTOR_COUNT) {
    return NULL;
  }
  return motor_states[motor_num].device;
}

void motor_deenergize_after(int motor_num, int timeout_ms) {
  if (motor_num < 0 || motor_num >= MOTOR_COUNT) {
    return;  // Invalid motor number
  }

  if (timeout_ms < 0) {
    // Negative value means always keep energized
    motor_states[motor_num].always_energized = true;
    motor_states[motor_num].idle_timeout_ticks =
        0;  // Unused when always_energized
  } else {
    // Convert milliseconds to ISR ticks
    motor_states[motor_num].always_energized = false;
    motor_states[motor_num].idle_timeout_ticks =
        (timeout_ms * 1000) / STEP_ISR_PERIOD_US;
  }
}

void motor_set_target_pos_drv(pos_drv_t target) {
  // Atomic update of all targets
  motor_states[0].target_steps = target.m0;
  motor_states[1].target_steps = target.m1;
  motor_states[2].target_steps = target.m2;
  motor_states[3].target_steps = target.m3;
  motor_states[4].target_steps = target.m4;
  motor_states[5].target_steps = target.m5;
  motor_states[6].target_steps = target.m6;
}

void motor_set_target_steps(int motor_num, int target_steps) {
  if (motor_num < 0 || motor_num >= MOTOR_COUNT) {
    return;  // Invalid motor number
  }
  motor_states[motor_num].target_steps = target_steps;
}

pos_drv_t motor_get_current_pos_drv() {
  // Read current positions
  return (pos_drv_t){.m0 = motor_states[0].current_steps,
                     .m1 = motor_states[1].current_steps,
                     .m2 = motor_states[2].current_steps,
                     .m3 = motor_states[3].current_steps,
                     .m4 = motor_states[4].current_steps,
                     .m5 = motor_states[5].current_steps,
                     .m6 = motor_states[6].current_steps};
}

void motor_dump_status() {
  char buf[256];
  const struct device* motors[] = {motor0, motor1, motor2, motor3,
                                   motor4, motor5, motor6};
  const char* names[] = {"mot0", "mot1", "mot2", "mot3",
                         "mot4", "mot5", "mot6"};

  for (int i = 0; i < MOTOR_COUNT; i++) {
    comm_print("%s: current_steps:%d energized:%s", names[i],
               motor_states[i].current_steps,
               motor_states[i].energized ? "true" : "false");
    int ret = tmc_dump_regs(motors[i], buf, sizeof(buf));
    if (ret < 0) {
      comm_print("%s: error %d", names[i], ret);
    } else {
      comm_print("%s: %s", names[i], buf);
    }
  }
}

void motor_run_steptest(int motor_num) {
  const struct device* motor = motor_get_device(motor_num);
  if (!motor) {
    comm_print_err("Invalid motor number: %d", motor_num);
    return;
  }

  comm_print("Running steptest on motor %d", motor_num);
  tmc_energize(motor, true);

  for (int i = 0; i < 2 * 200 * 32; i++) {  // 2 rotations at 32 microsteps
    // Check for cancel request
    if (g_cancel_requested) {
      comm_print("Steptest cancelled at step %d", i);
      break;
    }

    queue_step(motor_num, true);
    k_sleep(K_USEC(250));

    // Print SG_RESULT every 100 steps (50ms intervals at 500us/step)
    if (i % 100 == 0) {
      int sg_result = tmc_sgresult(motor);
      comm_print("SG:%d", sg_result);
    }

    if (tmc_stalled(motor)) {
      comm_print("Stall detected at step %d", i);
      break;
    }
  }

  tmc_energize(motor, false);
}

void motor_init() {
  // Initialize motor state arrays with default 200ms timeout
  uint32_t default_timeout_ticks = (200 * 1000) / STEP_ISR_PERIOD_US;
  const struct device* motors[] = {motor0, motor1, motor2, motor3,
                                   motor4, motor5, motor6};

  for (int i = 0; i < MOTOR_COUNT; i++) {
    motor_states[i] =
        (motor_step_state_t){.step_state = STEP_IDLE,
                             .device = motors[i],
                             .energized = false,
                             .idle_ticks = 0,
                             .idle_timeout_ticks = default_timeout_ticks,
                             .always_energized = false};
  }

  // Check motor devices
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (!device_is_ready(motors[i])) {
      comm_print_err("Motor%d device not ready", i);
      return;
    }
  }

  // Initialize step generation counter
  struct counter_top_cfg step_top_cfg = {
      .callback = step_tick_handler,
      .ticks = counter_us_to_ticks(step_gen_cnt,
                                   30),  // 30us ISR -> step pulse generation
  };

  counter_start(step_gen_cnt);
  int ret = counter_set_top_value(step_gen_cnt, &step_top_cfg);
  if (ret < 0) {
    comm_print_err("Step generation timer init failed: %d", ret);
    return;
  }
  comm_print("Step generation initialized");

  // Configure TCOOLTHRS for all motors
  for (int i = 0; i < MOTOR_COUNT; i++) {
    ret = tmc_set_tcoolthrs(motors[i], 750000);
    if (ret < 0) {
      comm_print_err("Failed to set TCOOLTHRS for motor %d", i);
    }
  }
}
