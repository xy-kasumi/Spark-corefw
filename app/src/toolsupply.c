// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "toolsupply.h"

#include "comm.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

static const struct device* pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const uint32_t pwm_channel = 0; /* PE9 is TIM1_CH1, but 0-indexed */

static float servo_on_ms_open = 0.0f;
static float servo_on_ms_closed = 100.0f;

static int set_servo(float on_ms) {
  uint32_t period = PWM_MSEC(20);  // 50Hz
  uint32_t on = PWM_USEC((int)(on_ms * 1000));
  return pwm_set(pwm_dev, pwm_channel, period, on, 0);
}

void toolsupply_init() {
  if (!device_is_ready(pwm_dev)) {
    comm_print("toolsupply: init failed (PWM failed)");
    return;
  }
  if (!set_servo(1.45)) {
    comm_print("toolsupply: init failed (PWM failed)");
    return;
  }
  comm_print("toolsupply: init ok");
}

void configure_tool_supply_servo_on(tool_supply_state_t state, float pos) {
  switch (state) {
    case TOOL_SUPPLY_OPEN:
      servo_on_ms_open = pos;
      break;
    case TOOL_SUPPLY_CLOSED:
      servo_on_ms_closed = pos;
      break;
  }
}

void set_tool_supply_state(tool_supply_state_t target) {
  float on_ms =
      (target == TOOL_SUPPLY_OPEN) ? servo_on_ms_open : servo_on_ms_closed;
  set_servo(on_ms);
  k_sleep(K_MSEC(500));  // wait for movement completion
}
