// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Powercore EDM module driver.
// Implements pulser.h interface using EN (GPIO output) + FEEDBACK_PWM (GPIO
// input with interrupt-based duty cycle measurement).
#include "pulser.h"

#include "canceler.h"
#include "comm.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// EN pin: PB8 (GPIO output, active high)
// FEEDBACK_PWM pin: PB9 (GPIO input, ~1kHz PWM)
static const struct device* gpiob_dev = DEVICE_DT_GET(DT_NODELABEL(gpiob));
#define EN_PIN 8
#define FEEDBACK_PIN 9

static bool init_success = false;
static bool energized = false;

// PWM measurement state (updated from ISR)
static volatile uint32_t rising_time_us;
static volatile uint32_t period_us;
static volatile uint32_t high_us;

// Derived feedback (0-255 scale, updated from work handler)
static uint8_t last_duty_255 = 255;  // 255 = fully open (no discharge)

static uint32_t poll_count = 0;

static struct gpio_callback feedback_cb_data;

static void feedback_edge_isr(const struct device* dev,
                              struct gpio_callback* cb,
                              uint32_t pins) {
  uint32_t now = k_cyc_to_us_floor32(k_cycle_get_32());
  int val = gpio_pin_get(gpiob_dev, FEEDBACK_PIN);
  if (val) {
    // Rising edge: compute period from last rising
    uint32_t prev = rising_time_us;
    rising_time_us = now;
    if (prev != 0) {
      period_us = now - prev;
    }
  } else {
    // Falling edge: compute high time
    high_us = now - rising_time_us;
  }
}

static struct k_work_delayable feedback_work;

// (system workqueue) Periodically compute duty cycle from ISR data and handle
// cancel.
static void feedback_work_handler(struct k_work* work) {
  k_work_reschedule(&feedback_work, K_MSEC(5));

  if (!energized) {
    last_duty_255 = 255;
    return;
  }

  if (canceler_cancel_needed()) {
    pulser_deenergize();
    return;
  }

  // Read ISR state
  uint32_t p = period_us;
  uint32_t h = high_us;
  if (p > 0) {
    uint32_t duty = (h * 255) / p;
    if (duty > 255) {
      duty = 255;
    }
    last_duty_255 = (uint8_t)duty;
  }

  poll_count++;
}

bool pulser_init() {
  if (!device_is_ready(gpiob_dev)) {
    comm_ps_k_vbool(PS_INIT, "pulser.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pulser.msg", "GPIOB not ready");
    return false;
  }

  // Configure EN as output (initially low / disabled)
  int ret = gpio_pin_configure(gpiob_dev, EN_PIN, GPIO_OUTPUT_INACTIVE);
  if (ret != 0) {
    comm_ps_k_vbool(PS_INIT, "pulser.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pulser.msg", "EN pin config failed");
    return false;
  }

  // Configure FEEDBACK as input with both-edge interrupt
  ret = gpio_pin_configure(gpiob_dev, FEEDBACK_PIN, GPIO_INPUT);
  if (ret != 0) {
    comm_ps_k_vbool(PS_INIT, "pulser.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pulser.msg", "feedback pin config failed");
    return false;
  }

  ret =
      gpio_pin_interrupt_configure(gpiob_dev, FEEDBACK_PIN, GPIO_INT_EDGE_BOTH);
  if (ret != 0) {
    comm_ps_k_vbool(PS_INIT, "pulser.ok", false);
    comm_ps_k_vfmtstr(PS_INIT, "pulser.msg", "feedback IRQ config failed");
    return false;
  }

  gpio_init_callback(&feedback_cb_data, feedback_edge_isr, BIT(FEEDBACK_PIN));
  gpio_add_callback(gpiob_dev, &feedback_cb_data);

  init_success = true;

  k_work_init_delayable(&feedback_work, feedback_work_handler);
  k_work_reschedule(&feedback_work, K_NO_WAIT);

  comm_ps_k_vbool(PS_INIT, "pulser.ok", true);
  comm_ps_k_vfmtstr(PS_INIT, "pulser.msg", "powercore");
  return true;
}

void pulser_dump_status() {
  if (!init_success) {
    comm_ps_k_vfmtstr(PS_STAT, "pulser.status", "init failed");
    return;
  }

  comm_ps_k_vbool(PS_STAT, "pulser.energized", energized);
  comm_ps_k_vint(PS_STAT, "pulser.poll_count", poll_count);
  comm_ps_k_vfloat(PS_STAT, "pulser.feedback_duty",
                   last_duty_255 * (1 / 255.0f));
  comm_ps_k_vint(PS_STAT, "pulser.feedback_period_us", period_us);
}

void pulser_energize(bool negative,
                     float pulse_us,
                     float current_a,
                     float duty_pct) {
  if (!init_success) {
    return;
  }

  // Powercore: parameters are configured on-board, just enable
  (void)negative;
  (void)pulse_us;
  (void)current_a;
  (void)duty_pct;

  gpio_pin_set(gpiob_dev, EN_PIN, 1);
  energized = true;
}

void pulser_deenergize() {
  if (!init_success) {
    return;
  }

  energized = false;
  last_duty_255 = 255;
  gpio_pin_set(gpiob_dev, EN_PIN, 0);
}

uint8_t pulser_get_short_rate() {
  return 255 - last_duty_255;
}

uint8_t pulser_get_pulse_rate() {
  return 0;
}

uint8_t pulser_get_open_rate() {
  return last_duty_255;
}

uint8_t pulser_get_temp() {
  return 0;
}

bool pulser_has_discharge() {
  return last_duty_255 < 255;
}
