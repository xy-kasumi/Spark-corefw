// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "pulser.h"

#include "comm.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

// Register addresses
#define PULSER_I2C_ADDR 0x3b
#define REG_POLARITY 0x01
#define REG_PULSE_CURRENT 0x02
#define REG_TEMPERATURE 0x03
#define REG_PULSE_DUR 0x04
#define REG_MAX_DUTY 0x05

// I2C device from device tree
static const struct device* i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

// Gate GPIO from device tree
static const struct gpio_dt_spec gate_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(pulser, gate), gpios);

// Status tracking
static bool init_success = false;
static uint32_t poll_count = 0;

// Read single register from pulser board
static bool read_register(uint8_t reg_addr, uint8_t* value) {
  if (!i2c_dev) {
    return false;
  }

  int ret = i2c_reg_read_byte(i2c_dev, PULSER_I2C_ADDR, reg_addr, value);
  poll_count++;
  return (ret == 0);
}

// Write single register to pulser board
static bool write_register(uint8_t reg_addr, uint8_t value) {
  if (!i2c_dev) {
    return false;
  }

  int ret = i2c_reg_write_byte(i2c_dev, PULSER_I2C_ADDR, reg_addr, value);
  return (ret == 0);
}

// Set gate GPIO state
static void set_gate(bool on) {
  gpio_pin_set_dt(&gate_gpio, on);
}

void pulser_init() {
  if (!i2c_dev) {
    comm_print("Pulser init: I2C device not found");
    return;
  }

  if (!device_is_ready(i2c_dev)) {
    comm_print("Pulser init: I2C device not ready");
    return;
  }

  if (!gpio_is_ready_dt(&gate_gpio)) {
    comm_print("Pulser init: Gate GPIO not ready");
    return;
  }

  int ret = gpio_pin_configure_dt(&gate_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    comm_print("Pulser init: Failed to configure gate GPIO");
    return;
  }

  init_success = true;
}

void pulser_dump_status() {
  comm_print("=== Pulser Status ===");
  comm_print("Init: %s", init_success ? "OK" : "FAIL");
  comm_print("I2C device: %s", i2c_dev ? "found" : "not found");
  comm_print("Poll count: %u", poll_count);

  if (init_success) {
    uint8_t temperature;
    if (read_register(REG_TEMPERATURE, &temperature)) {
      comm_print("Temperature: %u", temperature);
      comm_print("I2C: OK");
    } else {
      comm_print("I2C: FAIL (register read failed)");
    }
  } else {
    comm_print("I2C: SKIP (init failed)");
  }
}

void pulser_energize(bool negative,
                     float pulse_us,
                     float current_a,
                     float duty_pct) {
  if (!init_success) {
    comm_print("Pulser energize: Not initialized");
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
    comm_print("Pulser energize: I2C write failed");
    return;
  }

  // Enable gate
  set_gate(true);
  comm_print("Pulser energized: %s, %.0fµs, %.1fA, %.0f%%",
             negative ? "T-" : "T+", (double)pulse_us, (double)current_a,
             (double)duty_pct);
}

void pulser_deenergize() {
  if (!init_success) {
    comm_print("Pulser deenergize: Not initialized");
    return;
  }

  // Disable gate first
  set_gate(false);

  // Write polarity register to off
  bool ok = write_register(REG_POLARITY, 0);
  if (!ok) {
    comm_print("Pulser deenergize: I2C write failed");
    return;
  }

  comm_print("Pulser deenergized");
}
