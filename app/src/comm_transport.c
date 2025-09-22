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

static const struct device* uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static bool mode_interactive = true;

////////////////////
// RX (down)

// 2 should be enough.
// worst case: min valid payload (2 byte -> 20us at 100kbaud/s) coming in.
// Caller has (RX_QUEUE_CAPACITY - 1) x 20us time to consume the queue.
#define RX_QUEUE_CAPACITY 2

// empty (size == 0) data not allowed in tran_rx_msgq
K_MSGQ_DEFINE(tran_rx_msgq, sizeof(line_buf_t), RX_QUEUE_CAPACITY, 1);
static line_buf_t rx_assembly;

////////////////////
// TX (up)

#define TX_QUEUE_CAPACITY 2

// empty (size == 0) data not allowed in tran_tx_msgq
K_MSGQ_DEFINE(tran_tx_msgq, sizeof(line_buf_t), TX_QUEUE_CAPACITY, 1);
static line_buf_t tx_sending_buf;
static int tx_sending_pos = -1;  // -1 if tx_sending_buf is of no use.

// cf. CRC for 100byte is 20~40us.
// 1 byte transmit is 100us. should not calculate CRC for entire thing in this
// ISR. incremental CRC will be totally ok.

// (ISR) Send string for echoing.
static void echo_if_should(const char* data) {
  if (tx_sending_pos >= 0 || k_msgq_num_used_get(&tran_tx_msgq) > 0) {
    // shouldn't echo if sending something
    return;
  }
  line_buf_t buf;
  buf.size = strlen(data);
  memcpy(buf.buf, data, buf.size);
  k_msgq_put(&tran_tx_msgq, &buf, K_NO_WAIT);
}

// (ISR) UART interrupt handler
static void uart_isr(const struct device* dev, void* user_data) {
  uart_irq_update(dev);

  // RX
  if (uart_irq_rx_ready(dev)) {
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
      if (c == '\n') {
        // Newline: complete current buffer and notify
        if (mode_interactive) {
          echo_if_should("\r\n");
        }
        if (rx_assembly.size > 0) {
          k_msgq_put(&tran_rx_msgq, &rx_assembly, K_NO_WAIT);
          rx_assembly.size = 0;
        }
      } else if (c == '\b' || c == 0x7F) {
        // Backspace: pop last char & notify to handle backspace echo
        if (mode_interactive && rx_assembly.size > 0) {
          echo_if_should(" \b");
          rx_assembly.size--;
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

  // TX
  if (uart_irq_tx_ready(dev)) {
    if (tx_sending_pos < 0) {
      if (k_msgq_get(&tran_tx_msgq, &tx_sending_buf, K_NO_WAIT) == 0) {
        // start sending
        tx_sending_pos = 0;
      } else {
        // nothing to send
        uart_irq_tx_disable(dev);
        return;
      }
    }

    if (tx_sending_pos >= 0) {
      int to_send = tx_sending_buf.size - tx_sending_pos;
      int sent =
          uart_fifo_fill(dev, &tx_sending_buf.buf[tx_sending_pos], to_send);
      tx_sending_pos += sent;
      if (tx_sending_pos >= tx_sending_buf.size) {
        tx_sending_pos = -1;
      }
    }
  }
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

int tran_get_payload(line_buf_t* out, k_timeout_t timeout) {
  return k_msgq_get(&tran_rx_msgq, out, timeout);
}

void tran_put_payload(const uint8_t* data, int len) {
  // Silently truncate because too low layer to report error.
  if (len > 100) {
    len = 100;
  }

  line_buf_t buf;
  buf.size = len;
  memcpy(buf.buf, data, len);
  buf.buf[len] = '\n';
  buf.size = len + 1;

  uart_irq_tx_enable(uart_dev);
  k_msgq_put(&tran_tx_msgq, &buf, K_FOREVER);
}
