// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "pulser.h"

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

static bool energized = false;

// EDM state from latest poll
static uint8_t last_r_pulse = 0;
static uint8_t last_r_short = 0;
static uint8_t last_r_open = 0;

// Ring buffer for EDM polling data
#define EDM_BUFFER_SIZE 10000

typedef struct __attribute__((packed)) {
  uint8_t r_short;
  uint8_t r_open;
  uint8_t num_pulse;
  uint8_t reserved;
} edm_poll_entry_t;

static edm_poll_entry_t edm_buffer[EDM_BUFFER_SIZE];
static uint32_t edm_buffer_head = 0;   // Next write position
static uint32_t edm_buffer_count = 0;  // Number of entries stored

// Work queue for EDM status polling
static struct k_work edm_poll_work;
static struct k_timer edm_poll_timer;

// Atomic flag to prevent buffer writes during copy
static atomic_t copying_flag = ATOMIC_INIT(0);

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

// Work handler for EDM status polling (runs in system workqueue)
static void edm_poll_work_handler(struct k_work* work) {
  // Read REG_CKP_PS
  uint8_t val_ps = 0;
  int ret = i2c_reg_read_byte(i2c_dev, PULSER_I2C_ADDR, REG_CKP_PS, &val_ps);
  if (ret != 0) {
    return;
  }

  uint8_t val_p = (val_ps >> 4) & 0xf;
  uint8_t val_s = val_ps & 0xf;
  if (val_p + val_s > 15) {
    // should not happen according to the protcol. maybe data was corrupted due
    // to I2C noise. handle like comm failure.
    return;
  }

  // Update state from registers
  last_r_pulse = ((int)val_p * 255) / 15;
  last_r_short = ((int)val_s * 255) / 15;
  last_r_open = ((int)(15 - (val_p + val_s)) * 255) / 15;
  poll_count++;

  // Record (r_short, r_open, num_pulse) in ring buffer if not copying
  if (atomic_get(&copying_flag) == 0) {
    edm_buffer[edm_buffer_head].r_short = last_r_short;
    edm_buffer[edm_buffer_head].r_open = last_r_open;
    edm_buffer[edm_buffer_head].num_pulse = 0;
    edm_buffer[edm_buffer_head].reserved = 0;
    edm_buffer_head = (edm_buffer_head + 1) % EDM_BUFFER_SIZE;

    if (edm_buffer_count < EDM_BUFFER_SIZE) {
      edm_buffer_count++;
    }
  }
}

// Timer callback - schedules EDM polling work
static void edm_poll_timer_handler(struct k_timer* timer) {
  if (!init_success) {
    return;
  }
  if (!energized) {
    return;
  }
  k_work_submit(&edm_poll_work);
}

bool pulser_init() {
  if (!i2c_dev) {
    comm_ps_old_kv_fmt("pulser.ok", "false");
    comm_ps_old_kv_str("pulser.msg", "I2C device not found");
    return false;
  }

  if (!device_is_ready(i2c_dev)) {
    comm_ps_old_kv_fmt("pulser.ok", "false");
    comm_ps_old_kv_str("pulser.msg", "I2C device not ready");
    return false;
  }

  // Check comm by reading temperature.
  uint8_t temp = 0;
  if (!read_register(REG_TEMPERATURE, &temp)) {
    comm_ps_old_kv_fmt("pulser.ok", "false");
    comm_ps_old_kv_str("pulser.msg", "I2C read failed");
    return false;
  }

  // Initialize work item
  k_work_init(&edm_poll_work, edm_poll_work_handler);

  // Initialize and start polling timer (1ms period)
  k_timer_init(&edm_poll_timer, edm_poll_timer_handler, NULL);
  k_timer_start(&edm_poll_timer, K_MSEC(1), K_MSEC(1));

  init_success = true;
  comm_ps_old_kv_fmt("pulser.ok", "true");
  return true;
}

void pulser_dump_status() {
  if (!init_success) {
    comm_print("status: init failed");
    return;
  }

  comm_print("poll count: %u", poll_count);
  comm_print("EDM state: r_pulse=%u, r_short=%u, r_open=%u", last_r_pulse,
             last_r_short, last_r_open);
  comm_print("EDM buffer: %u/%u entries (%.1f%% full)", edm_buffer_count,
             EDM_BUFFER_SIZE,
             (double)(edm_buffer_count * 100) / EDM_BUFFER_SIZE);

  uint8_t temperature;
  if (read_register(REG_TEMPERATURE, &temperature)) {
    comm_print("temperature: %u", temperature);
    comm_print("status: ok");
  } else {
    comm_print("status: i2c read fail");
  }
}

void pulser_energize(bool negative,
                     float pulse_us,
                     float current_a,
                     float duty_pct) {
  if (!init_success) {
    comm_print_err("pulser: energize failed (not initialized)");
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
    comm_print_err("pulser: energize failed (I2C write failed)");
    return;
  }

  energized = true;
  comm_print("pulser: energized (%s, %.0fµs, %.1fA, %.0f%%)",
             negative ? "T-" : "T+", (double)pulse_us, (double)current_a,
             (double)duty_pct);
}

void pulser_deenergize() {
  if (!init_success) {
    return;
  }

  energized = false;

  // Write polarity register to off
  bool ok = write_register(REG_POLARITY, 0);
  if (!ok) {
    comm_print_err("pulser: deenergize failed (I2C write failed)");
    return;
  }

  comm_print("pulser: deenergized");
}

uint32_t pulser_get_buffer_count() {
  return edm_buffer_count;
}

bool pulser_get_buffer_entry(uint32_t index,
                             uint8_t* r_short,
                             uint8_t* r_open,
                             uint8_t* num_pulse) {
  if (index >= edm_buffer_count) {
    return false;
  }

  uint32_t actual_index;
  if (edm_buffer_count < EDM_BUFFER_SIZE) {
    // Buffer not full yet, entries are from 0 to edm_buffer_count-1
    actual_index = index;
  } else {
    // Buffer is full, oldest entry is at edm_buffer_head
    actual_index = (edm_buffer_head + index) % EDM_BUFFER_SIZE;
  }

  *r_short = edm_buffer[actual_index].r_short;
  *r_open = edm_buffer[actual_index].r_open;
  *num_pulse = edm_buffer[actual_index].num_pulse;
  return true;
}

void pulser_clear_buffer() {
  atomic_set(&copying_flag, 1);
  edm_buffer_head = 0;
  edm_buffer_count = 0;
  atomic_set(&copying_flag, 0);
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

uint32_t pulser_copy_log_to_buffer(uint8_t* buffer, uint32_t max_size) {
  // Set copying flag to prevent new writes during copy
  atomic_set(&copying_flag, 1);

  uint32_t entry_size = sizeof(edm_poll_entry_t);
  uint32_t max_entries = max_size / entry_size;
  uint32_t entries_to_copy =
      (edm_buffer_count < max_entries) ? edm_buffer_count : max_entries;

  for (uint32_t i = 0; i < entries_to_copy; i++) {
    uint32_t actual_index;
    if (edm_buffer_count < EDM_BUFFER_SIZE) {
      actual_index = i;
    } else {
      actual_index = (edm_buffer_head + i) % EDM_BUFFER_SIZE;
    }

    uint8_t* dest = buffer + (i * entry_size);
    edm_poll_entry_t* src = &edm_buffer[actual_index];

    dest[0] = src->r_short;
    dest[1] = src->r_open;
    dest[2] = src->num_pulse;
    dest[3] = src->reserved;
  }

  // Clear copying flag to resume writes
  atomic_set(&copying_flag, 0);
  return entries_to_copy * entry_size;
}
