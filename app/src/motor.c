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
static void step_tick() {
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
static void step_tick_handler(const struct device* dev, void* user_data) {
  step_tick();
}

void queue_step(bool dir) {
  if (dir) {
    remaining_steps++;
  } else {
    remaining_steps--;
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

void motor_read_registers() {
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

void motor_run_steptest() {
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

void motor_init() {
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
