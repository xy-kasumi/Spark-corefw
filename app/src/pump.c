// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "pump.h"

#include "comm.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

static const struct gpio_dt_spec pump_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), he1_gate_gpios);

bool pump_init() {
  if (!gpio_is_ready_dt(&pump_gpio)) {
    comm_ps_k_vbool(PS_INIT, "pump.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pump.msg", "GPIO device not ready");
    return false;
  }

  int ret = gpio_pin_configure_dt(&pump_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret != 0) {
    comm_ps_k_vbool(PS_INIT, "pump.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pump.msg", "GPIO config failed (code %d)", ret);
    return false;
  }

  comm_ps_k_vbool(PS_INIT, "pump.ok", true);
  return true;
}

void pump_set_enable(bool enable) {
  gpio_pin_set_dt(&pump_gpio, enable ? 1 : 0);

  if (enable) {
    k_sleep(K_MSEC(1000));  // Wait 1 sec after starting pump
  } else {
    k_sleep(K_MSEC(100));  // Wait 100 msec after stopping pump
  }
}
