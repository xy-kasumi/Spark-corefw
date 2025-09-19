// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "toolsupply.h"

#include "comm.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

static const struct device* pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const uint32_t pwm_channel = 1;

static float servo_on_ms_open = 1.3f;
static float servo_on_ms_closed = 1.6f;

static tool_supply_state_t current_state = TOOL_SUPPLY_CLOSED;
static float current_servo_on_ms = 1.6f;

static inline float lerp(float a, float b, float t) {
  return a + t * (b - a);
}

/**
 * @returns 0 on success, error code otherwise.
 */
static int set_servo(float on_ms) {
  uint32_t period = PWM_MSEC(20);  // 50Hz
  uint32_t on = PWM_USEC((int)(on_ms * 1000));
  return pwm_set(pwm_dev, pwm_channel, period, on, 0);
}

bool toolsupply_init() {
  if (!device_is_ready(pwm_dev)) {
    comm_ps_k_vbool(PS_INIT, "toolsupply.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "toolsupply.msg", "PWM device not ready");
    return false;
  }
  int ret = set_servo(current_servo_on_ms);
  if (ret) {
    comm_ps_k_vbool(PS_INIT, "toolsupply.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "toolsupply.msg", "PWM failed (code %d)", ret);
    return false;
  }
  comm_ps_k_vbool(PS_INIT, "toolsupply.ok", true);
  return true;
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
  // Apply potentially changed target position.
  set_tool_supply_state(current_state);
}

void set_tool_supply_state(tool_supply_state_t target) {
  const int num_cycles = 100;
  float src = current_servo_on_ms;
  float dst =
      (target == TOOL_SUPPLY_OPEN) ? servo_on_ms_open : servo_on_ms_closed;

  for (int cycle = 1; cycle <= num_cycles; cycle++) {
    float t = (float)cycle / (float)num_cycles;
    float on_ms = lerp(src, dst, t);
    set_servo(on_ms);
    k_sleep(K_MSEC(10));
  }
  current_servo_on_ms = dst;
  current_state = target;
}
