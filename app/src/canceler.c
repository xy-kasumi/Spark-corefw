// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "canceler.h"

#include <zephyr/kernel.h>

static struct k_timer cancel_timer;
static bool cancel_needed = false;

// (ISR) Handler to end cancelation window
static void on_cancel_end(struct k_timer* timer) {
  cancel_needed = false;
}

void canceler_init() {
  k_timer_init(&cancel_timer, &on_cancel_end, NULL);
  cancel_needed = false;
}

void canceler_cancel() {
  cancel_needed = true;
  k_timer_start(&cancel_timer, CANCEL_TIMEOUT, K_FOREVER);
}

bool canceler_cancel_needed() {
  return cancel_needed;
}
