// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "pulser.h"

#include "canceler.h"
#include "comm.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

// I2C address
#define PULSER_I2C_ADDR 0x3b

// Registers (from
// https://github.com/xy-kasumi/Spark/blob/main/docs/user-PULSER.md)
#define REG_POLARITY 0x01       // RW: 0=OFF, 1-4=energize with polarity
#define REG_PULSE_CURRENT 0x02  // RW: pulse current in 100mA units (1-200)
#define REG_TEMPERATURE 0x03    // R:  heatsink temperature in °C
#define REG_PULSE_DUR 0x04      // RW: pulse duration in 10us units (5-100)
#define REG_MAX_DUTY 0x05       // RW: max duty factor in percent (1-95)
#define REG_CKP_PS 0x10         // R (special): rate of pulse & short

// I2C device from device tree
static const struct device* i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

// Status tracking
static bool init_success = false;
static uint32_t poll_count = 0;
static uint32_t num_i2c_fail = 0;

static bool energized = false;

// EDM state from latest poll
static uint8_t last_r_pulse = 0;
static uint8_t last_r_short = 0;
static uint8_t last_r_open = 0;

static struct k_work_delayable edm_poll_work;

// Read single register from pulser board, returns true on success
static bool read_register(uint8_t reg_addr, uint8_t* value) {
  if (!i2c_dev) {
    return false;
  }

  int ret = i2c_reg_read_byte(i2c_dev, PULSER_I2C_ADDR, reg_addr, value);
  poll_count++;
  return (ret == 0);
}

// Write single register to pulser board, returns true on success
static bool write_register(uint8_t reg_addr, uint8_t value) {
  if (!i2c_dev) {
    return false;
  }

  int ret = i2c_reg_write_byte(i2c_dev, PULSER_I2C_ADDR, reg_addr, value);
  return (ret == 0);
}

// (system workqueue) Work handler for EDM status polling & cancelation.
static void edm_poll_work_handler(struct k_work* work) {
  k_work_reschedule(&edm_poll_work, K_MSEC(1));
  if (!energized) {
    return;
  }
  if (canceler_cancel_needed()) {
    pulser_deenergize();
    return;
  }

  // Read REG_CKP_PS
  uint8_t val_ps = 0;
  int ret = i2c_reg_read_byte(i2c_dev, PULSER_I2C_ADDR, REG_CKP_PS, &val_ps);
  if (ret != 0) {
    num_i2c_fail++;
    return;
  }

  uint8_t val_p = (val_ps >> 4) & 0xf;
  uint8_t val_s = val_ps & 0xf;
  if (val_p + val_s > 15) {
    // should not happen according to the protcol. maybe data was corrupted due
    // to I2C noise. handle like comm failure.
    num_i2c_fail++;
    return;
  }

  // Update state from registers
  last_r_pulse = ((int)val_p * 255) / 15;
  last_r_short = ((int)val_s * 255) / 15;
  last_r_open = ((int)(15 - (val_p + val_s)) * 255) / 15;
  poll_count++;
}

bool pulser_init() {
  if (!device_is_ready(i2c_dev)) {
    comm_ps_k_vbool(PS_INIT, "pulser.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pulser.msg", "I2C device not ready");
    return false;
  }

  // Check comm by reading temperature.
  uint8_t temp = 0;
  if (!read_register(REG_TEMPERATURE, &temp)) {
    comm_ps_k_vbool(PS_INIT, "pulser.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pulser.msg", "I2C read failed");
    return false;
  }

  init_success = true;

  // Initialize work item
  k_work_init_delayable(&edm_poll_work, edm_poll_work_handler);
  k_work_reschedule(&edm_poll_work, K_NO_WAIT);

  comm_ps_k_vbool(PS_INIT, "pulser.ok", true);
  return true;
}

void pulser_dump_status() {
  if (!init_success) {
    comm_ps_k_vfmtstr(PS_STAT, "pulser.status", "init failed");
    return;
  }

  comm_ps_k_vbool(PS_STAT, "pulser.energized", energized);
  comm_ps_k_vint(PS_STAT, "pulser.poll_count", poll_count);
  comm_ps_k_vint(PS_STAT, "pulser.i2c_fail", num_i2c_fail);
  comm_ps_k_vfloat(PS_STAT, "pulser.edm.r_pulse", last_r_pulse * (1 / 255.0f));
  comm_ps_k_vfloat(PS_STAT, "pulser.edm.r_short", last_r_short * (1 / 255.0f));
  comm_ps_k_vfloat(PS_STAT, "pulser.edm.r_open", last_r_open * (1 / 255.0f));

  uint8_t temperature;
  if (read_register(REG_TEMPERATURE, &temperature)) {
    comm_ps_k_vint(PS_STAT, "pulser.temp_c", temperature);
  } else {
    comm_ps_k_vfmtstr(PS_STAT, "pulser.temp_c", "error");
  }

  uint8_t curr_100ma;
  if (read_register(REG_PULSE_CURRENT, &curr_100ma)) {
    comm_ps_k_vfloat(PS_STAT, "pulser.pulse_current_a", curr_100ma * 0.1f);
  } else {
    comm_ps_k_vfmtstr(PS_STAT, "pulser.pulse_current_a", "error");
  }

  uint8_t dur_10us;
  if (read_register(REG_PULSE_DUR, &dur_10us)) {
    comm_ps_k_vfloat(PS_STAT, "pulser.pulse_dur_us", dur_10us * 10.0f);
  } else {
    comm_ps_k_vfmtstr(PS_STAT, "pulser.pulse_dur_us", "error");
  }

  uint8_t duty_pct;
  if (read_register(REG_MAX_DUTY, &duty_pct)) {
    comm_ps_k_vfloat(PS_STAT, "pulser.max_duty_pct", duty_pct);
  } else {
    comm_ps_k_vfmtstr(PS_STAT, "pulser.max_duty_pct", "error");
  }
}

void pulser_energize(bool negative,
                     float pulse_us,
                     float current_a,
                     float duty_pct) {
  if (!init_success) {
    return;
  }

  // Convert parameters to register values (from plugin_edm.c)
  uint8_t pulse_dur_10us =
      (uint8_t)(pulse_us * 0.1f);  // Convert µs to 10µs units
  uint8_t pulse_current_100ma =
      (uint8_t)(current_a * 10.0f);  // Convert A to 100mA units
  uint8_t pulse_duty_pct = (uint8_t)duty_pct;
  uint8_t polarity = negative ? 2 : 1;  // 2: T- W+, 1: T+ W-

  // Ensure minimum current
  if (pulse_current_100ma == 0) {
    pulse_current_100ma = 1;  // 100mA minimum
  }

  // Write registers
  bool all_ok = true;
  all_ok &= write_register(REG_PULSE_CURRENT, pulse_current_100ma);
  all_ok &= write_register(REG_PULSE_DUR, pulse_dur_10us);
  all_ok &= write_register(REG_MAX_DUTY, pulse_duty_pct);
  all_ok &= write_register(REG_POLARITY, polarity);

  if (!all_ok) {
    // CM:comm_print_err("pulser: energize failed (I2C write failed)");
    return;
  }

  energized = true;
  comm_ps_raw(PS_ERROR, "< pulser.energized:true >");
}

void pulser_deenergize() {
  if (!init_success) {
    return;
  }

  energized = false;

  // Write polarity register to off
  bool ok = write_register(REG_POLARITY, 0);
  if (!ok) {
    // CM:comm_print_err("pulser: deenergize failed (I2C write failed)");
    return;
  }

  comm_ps_raw(PS_ERROR, "< pulser.energized:false >");
}

uint8_t pulser_get_short_rate() {
  return last_r_short;
}

uint8_t pulser_get_pulse_rate() {
  return last_r_pulse;
}

uint8_t pulser_get_open_rate() {
  return last_r_open;
}

bool pulser_has_discharge() {
  return (last_r_pulse > 0 || last_r_short > 0);
}
