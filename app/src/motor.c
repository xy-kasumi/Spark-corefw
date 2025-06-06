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

// Step pulse generation state machine
typedef enum {
  STEP_IDLE,        // No stepping in progress
  STEP_PULSE_HIGH,  // Step pin is HIGH (1 tick)
  STEP_PULSE_LOW,   // Step pin is LOW, waiting before next step (1 tick)
} step_state_t;

// Per-motor step generation state
typedef struct {
  volatile int target_steps;    // Target position in microsteps
  volatile int current_steps;   // Current position in microsteps
  bool current_direction;       // Current direction state
  step_state_t step_state;      // Current state machine state
  const struct device* device;  // Motor device reference
} motor_step_state_t;

static motor_step_state_t motor_states[3];

// Process step generation for a single motor
static void process_motor_step(motor_step_state_t* motor) {
  switch (motor->step_state) {
    case STEP_IDLE:
      if (motor->current_steps != motor->target_steps) {
        // Need to step toward target
        bool dir = (motor->target_steps > motor->current_steps);
        if (dir != motor->current_direction) {
          motor->current_direction = dir;
          tmc_set_dir(motor->device, dir);
        }

        // Start step pulse (HIGH)
        tmc_set_step(motor->device, true);
        motor->step_state = STEP_PULSE_HIGH;
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
  for (int i = 0; i < 3; i++) {
    process_motor_step(&motor_states[i]);
  }
}

void queue_step(int motor_num, bool dir) {
  if (motor_num < 0 || motor_num >= 3) {
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
  switch (motor_num) {
    case 0:
      return motor0;
    case 1:
      return motor1;
    case 2:
      return motor2;
    default:
      return NULL;
  }
}

void motor_energize(int motor_num, bool enable) {
  const struct device* motor = motor_get_device(motor_num);
  if (motor) {
    tmc_energize(motor, enable);
  }
}

void motor_set_target_pos_drv(pos_drv_t target) {
  // Atomic update of all three targets
  motor_states[0].target_steps = target.m0;
  motor_states[1].target_steps = target.m1;
  motor_states[2].target_steps = target.m2;
}

pos_drv_t motor_get_current_pos_drv() {
  // Read current positions
  return (pos_drv_t){.m0 = motor_states[0].current_steps,
                     .m1 = motor_states[1].current_steps,
                     .m2 = motor_states[2].current_steps};
}

void motor_dump_registers() {
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
      comm_print_info("Steptest cancelled at step %d", i);
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
  // Initialize motor state arrays
  motor_states[0] =
      (motor_step_state_t){.step_state = STEP_IDLE, .device = motor0};
  motor_states[1] =
      (motor_step_state_t){.step_state = STEP_IDLE, .device = motor1};
  motor_states[2] =
      (motor_step_state_t){.step_state = STEP_IDLE, .device = motor2};

  // Check motor devices
  if (!device_is_ready(motor0)) {
    comm_print_err("Motor0 device not ready");
    return;
  }
  if (!device_is_ready(motor1)) {
    comm_print_err("Motor1 device not ready");
    return;
  }
  if (!device_is_ready(motor2)) {
    comm_print_err("Motor2 device not ready");
    return;
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
  const struct device* motors[] = {motor0, motor1, motor2};
  for (int i = 0; i < 3; i++) {
    ret = tmc_set_tcoolthrs(motors[i], 750000);
    if (ret < 0) {
      comm_print_err("Failed to set TCOOLTHRS for motor %d", i);
    }
  }
}
