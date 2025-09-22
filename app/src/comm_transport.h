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

// Maximum valid line size allowed by the protocol.
// payload + 1 (seq) + 4 (hash) + 1 (newline)
#define LINE_BUFFER_SIZE (PAYLOAD_BUFFER_SIZE + 6)

typedef struct {
  int size;
  uint8_t buf[LINE_BUFFER_SIZE];
} line_buf_t;

// RX event definitions
extern struct k_event tran_rx_events;

/**
 * One or more payload became available in tran_rx_msgq.
 * Caller should exhaust a queue on this event (because event might get missed
 * when busy).
 */
#define RX_EVENT_PAYLOAD_RECEIVED BIT(0)

/**
 * Backspace was received. (caller might want to send echo)
 */
#define RX_EVENT_BACKSPACE BIT(1)

// RX buffer
extern struct k_msgq tran_rx_msgq;

/**
 * (blocking) Init transport layer.
 * @returns true on success.
 */
bool tran_init();

/**
 * Write data. Caller is free to reuse data for other purposes.
 * This function will block until transmit is complete and internal buffer is
 * ready.
 *
 * @param data data to transmit
 * @param len Length of data (must be <= 256 bytes, excess will be silently
 * truncated)
 */
void tran_uart_write(const uint8_t* data, int len);
