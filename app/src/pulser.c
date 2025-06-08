// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "pulser.h"

#include "comm.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

// I2C address
#define PULSER_I2C_ADDR 0x3b

// Registers (from
// https://github.com/xy-kasumi/Spark/blob/main/docs/user-PULSER.md)
#define REG_POLARITY 0x01       // RW: 0=OFF, 1-4=energize with polarity
#define REG_PULSE_CURRENT 0x02  // RW: pulse current in 100mA units (1-200)
#define REG_TEMPERATURE 0x03    // R:  heatsink temperature in °C
#define REG_PULSE_DUR 0x04      // RW: pulse duration in 10us units (5-100)
#define REG_MAX_DUTY 0x05       // RW: max duty factor in percent (1-95)
#define REG_CKP_N_PULSE 0x10    // R:  number of pulses (checkpoint read)
#define REG_T_IGNITION 0x11     // R:  avg ignition time in 5us units
#define REG_T_IGNITION_SD 0x12  // R:  std dev of ignition time in 5us units
#define REG_R_PULSE 0x13        // R:  ratio spent discharging (0-255)
#define REG_R_SHORT 0x14        // R:  ratio spent shorted (0-255)
#define REG_R_OPEN 0x15         // R:  ratio spent waiting (0-255)

// I2C device from device tree
static const struct device* i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

// Gate GPIO from device tree
static const struct gpio_dt_spec gate_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(pulser, gate), gpios);

// Status tracking
static bool init_success = false;
static uint32_t poll_count = 0;

// EDM state from latest poll
static uint8_t last_r_pulse = 0;
static uint8_t last_r_short = 0;
static uint8_t last_r_open = 0;
static uint8_t last_n_pulse = 0;

// Work queue for EDM status polling
static struct k_work edm_poll_work;
static struct k_timer edm_poll_timer;

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

// Work handler for EDM status polling (runs in system workqueue)
static void edm_poll_work_handler(struct k_work* work) {
  if (!init_success) {
    return;
  }

  // Read 6 registers starting from REG_CKP_N_PULSE
  uint8_t buf[REG_R_OPEN - REG_CKP_N_PULSE + 1];
  int ret = i2c_burst_read(i2c_dev, PULSER_I2C_ADDR, REG_CKP_N_PULSE, buf, 6);
  if (ret != 0) {
    return;
  }

  // Update state from registers
  last_n_pulse = buf[REG_CKP_N_PULSE - REG_CKP_N_PULSE];
  last_r_pulse = buf[REG_R_PULSE - REG_CKP_N_PULSE];
  last_r_short = buf[REG_R_SHORT - REG_CKP_N_PULSE];
  last_r_open = buf[REG_R_OPEN - REG_CKP_N_PULSE];
  poll_count++;
}

// Timer callback - schedules EDM polling work
static void edm_poll_timer_handler(struct k_timer* timer) {
  k_work_submit(&edm_poll_work);
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

  // Initialize work item
  k_work_init(&edm_poll_work, edm_poll_work_handler);

  // Initialize and start polling timer (1ms period)
  k_timer_init(&edm_poll_timer, edm_poll_timer_handler, NULL);
  k_timer_start(&edm_poll_timer, K_MSEC(1), K_MSEC(1));

  init_success = true;
  comm_print("Pulser initialized with 1ms polling via workqueue");
}

void pulser_dump_status() {
  comm_print("=== Pulser Status ===");
  comm_print("Init: %s", init_success ? "OK" : "FAIL");
  comm_print("I2C device: %s", i2c_dev ? "found" : "not found");
  comm_print("Poll count: %u", poll_count);
  comm_print("EDM state: n_pulse=%u, r_pulse=%u, r_short=%u, r_open=%u",
             last_n_pulse, last_r_pulse, last_r_short, last_r_open);

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

uint8_t pulser_get_short_rate() {
  return last_r_short;
}

uint8_t pulser_get_open_rate() {
  return last_r_open;
}

bool pulser_has_discharge() {
  return (last_r_pulse > 0 || last_r_short > 0);
}
