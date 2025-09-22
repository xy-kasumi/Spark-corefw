// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "comm_transport.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

// UART device
static const struct device* uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

K_EVENT_DEFINE(tran_rx_events);

// 2 should be enough.
// worst case: min valid payload (2 byte -> 20us at 100kbaud/s) coming in.
// Caller has (RX_QUEUE_CAPACITY - 1) x 20us time to consume the queue.
#define RX_QUEUE_CAPACITY 2

// RX (down)
K_MSGQ_DEFINE(tran_rx_msgq, sizeof(rx_buf_t), RX_QUEUE_CAPACITY, 1);
static rx_buf_t rx_assembly;

// TX buffer and state
static uint8_t tx_buffer[256];
static K_MUTEX_DEFINE(tx_mutex);
static volatile int tx_len = 0;
static volatile int tx_pos = 0;
static K_SEM_DEFINE(tx_done, 1, 1);

// cf. CRC for 100byte is 20~40us.
// 1 byte transmit is 100us. should not calculate CRC for entire thing in this
// ISR. incremental CRC will be totally ok.

// UART interrupt handler
static void uart_isr(const struct device* dev, void* user_data) {
  uart_irq_update(dev);

  // RX handling - directly into command_buffer
  if (uart_irq_rx_ready(dev)) {
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
      if (c == '\n') {
        // Newline: complete current buffer and notify
        if (rx_assembly.size > 0) {
          k_msgq_put(&tran_rx_msgq, &rx_assembly, K_NO_WAIT);
          rx_assembly.size = 0;
          k_event_post(&tran_rx_events, RX_EVENT_PAYLOAD_RECEIVED);
        }
      } else if (c == '\b' || c == 0x7F) {
        // Backspace: pop last char & notify to handle backspace echo
        if (rx_assembly.size > 0) {
          rx_assembly.size--;
          k_event_post(&tran_rx_events, RX_EVENT_BACKSPACE);
        }
      } else if (c >= 0x20 && c <= 0x7E) {
        // Valid line content
        if (rx_assembly.size < LINE_BUFFER_SIZE) {
          rx_assembly.buf[rx_assembly.size] = c;
          rx_assembly.size++;
        }
      }
    }
  }

  // TX handling - from static buffer
  if (uart_irq_tx_ready(dev) && tx_pos < tx_len) {
    int to_send = tx_len - tx_pos;
    int sent = uart_fifo_fill(dev, &tx_buffer[tx_pos], to_send);
    tx_pos += sent;

    if (tx_pos >= tx_len) {
      uart_irq_tx_disable(dev);
      k_sem_give(&tx_done);
    }
  }
}

void tran_uart_write(const uint8_t* data, int len) {
  // Silently truncate because too low layer to report error.
  if (len > 100) {
    len = 100;
  }
  k_mutex_lock(&tx_mutex, K_FOREVER);
  k_sem_take(&tx_done, K_FOREVER);

  // Copy to ISR buffer.
  memcpy(tx_buffer, data, len);
  tx_buffer[len] = '\n';
  tx_len = len + 1;
  tx_pos = 0;
  uart_irq_tx_enable(uart_dev);

  // Wait for completion before releasing mutex
  k_sem_take(&tx_done, K_FOREVER);
  k_sem_give(&tx_done);  // Reset for next use
  k_mutex_unlock(&tx_mutex);
}

bool tran_init() {
  if (!device_is_ready(uart_dev)) {
    // can't do any reporting
    return false;
  }

  // Configure UART interrupts
  uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
  uart_irq_rx_enable(uart_dev);
  return true;
}
