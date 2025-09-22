// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Internal layer for comm module, handling transport layer of the
 * protocol.
 */
#pragma once

#include "slice.h"

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

// Maximum payload size defined by the protocol.
#define PAYLOAD_BUFFER_SIZE 100

typedef struct {
  int size;
  uint8_t data[PAYLOAD_BUFFER_SIZE];
} payload_t;

/**
 * Init transport layer.
 * @returns true on success.
 */
bool tran_init();

/**
 * Get received payload.
 * @param out output buffer. Guaranteed to have size > 0 on success.
 * @param timeout timeout
 * @returns 0 if data is copied to out. -ENOMSG if no message (when K_NO_WAIT).
 * -EAGAIN timed out.
 */
int tran_get_payload(payload_t* out, k_timeout_t timeout);

/**
 * Configures poll event for tran_get_payload() availability.
 */
void tran_poll_event_get(struct k_poll_event* event);

/**
 * Write data. Caller is free to reuse data for other purposes.
 * This function will block until transmit is complete and internal buffer is
 * ready.
 */
void tran_put_payload(const payload_t* payload);
