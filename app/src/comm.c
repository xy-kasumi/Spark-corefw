// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "comm.h"

#include "system.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

// UART device
static const struct device* uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

// Line ending for output
static const uint8_t LINE_ENDING[] = "\r\n";
#define LINE_ENDING_LEN 2

// RX buffer and state
static char command_buffer[256];
static volatile int rx_pos = 0;
static K_EVENT_DEFINE(rx_events);

// RX event definitions
#define RX_EVENT_COMMAND_RECEIVED BIT(0)
#define RX_EVENT_BACKSPACE BIT(1)

// TX buffer and state
static uint8_t tx_buffer[256];
static K_MUTEX_DEFINE(tx_mutex);
static volatile int tx_len = 0;
static volatile int tx_pos = 0;
static K_SEM_DEFINE(tx_done, 1, 1);

// Recv buffer
static K_MUTEX_DEFINE(rbuf_mutex);
payload_t recv_buffer;
int recv_buffer_num = 0;

// UART interrupt handler
static void uart_isr(const struct device* dev, void* user_data) {
  uart_irq_update(dev);

  // RX handling - directly into command_buffer
  if (uart_irq_rx_ready(dev)) {
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
      if (c == '\r' || c == '\n') {
        // Can accept any of CR, CRLF, LF.
        if (rx_pos == 1 && command_buffer[0] == '!') {
          // Special case: cancel
          if (state_machine_get_state() != STATE_IDLE) {
            g_cancel_requested = true;
          }
          // Reset command buffer
          rx_pos = 0;
        } else if (rx_pos > 0) {
          command_buffer[rx_pos] = '\0';
          rx_pos = 0;
          k_event_post(&rx_events, RX_EVENT_COMMAND_RECEIVED);
        }
      } else if (c == '\b' || c == 0x7F) {
        // Backspace handling - signal event for thread to handle echo
        if (rx_pos > 0) {
          rx_pos--;
          k_event_post(&rx_events, RX_EVENT_BACKSPACE);
        }
      } else if (c >= 0x20 && c <= 0x7E) {
        // Printable character or cancel command
        if (rx_pos < sizeof(command_buffer) - 1) {
          command_buffer[rx_pos] = c;
          rx_pos++;
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

/**
 * Safe UART write with mutex protection
 * @param data Binary data to transmit
 * @param len Length of data (must be <= 256 bytes, excess will be silently
 * truncated)
 */
static void uart_write(const uint8_t* data, int len) {
  k_mutex_lock(&tx_mutex, K_FOREVER);
  k_sem_take(&tx_done, K_FOREVER);

  // Copy to safe buffer - silently truncate if too long
  len = (len > sizeof(tx_buffer)) ? sizeof(tx_buffer) : len;
  memcpy(tx_buffer, data, len);
  tx_len = len;
  tx_pos = 0;
  uart_irq_tx_enable(uart_dev);

  // Wait for completion before releasing mutex
  k_sem_take(&tx_done, K_FOREVER);
  k_sem_give(&tx_done);  // Reset for next use
  k_mutex_unlock(&tx_mutex);
}

static void uart_puts(const char* str) {
  uart_write((const uint8_t*)str, strlen(str));
}

static void comm_thread(void* p1, void* p2, void* p3) {
  while (1) {
    // Wait for RX events
    uint32_t events =
        k_event_wait(&rx_events, RX_EVENT_COMMAND_RECEIVED | RX_EVENT_BACKSPACE,
                     false, K_FOREVER);

    if (events & RX_EVENT_BACKSPACE) {
      k_event_clear(&rx_events, RX_EVENT_BACKSPACE);

      uart_write((const uint8_t*)" \b", 2);  // backspace echo
      continue;
    }

    if (events & RX_EVENT_COMMAND_RECEIVED) {
      k_event_clear(&rx_events, RX_EVENT_COMMAND_RECEIVED);

      uart_write(LINE_ENDING, LINE_ENDING_LEN);  // echo new line

      // Trim leading whitespace
      char* trimmed = command_buffer;
      while (*trimmed == ' ' || *trimmed == '\t') {
        trimmed++;
      }

      // Only accept commands in IDLE state
      if (g_machine_state != STATE_IDLE) {
        continue;  // Silently ignore
      }

      // Copy command to caller's buffer
      k_mutex_lock(&rbuf_mutex, K_FOREVER);
      if (recv_buffer_num == 0) {
        strncpy(recv_buffer.data, trimmed, sizeof(recv_buffer.data));
        recv_buffer_num++;
      } else {
        // silently drop when buffer is full.
      }
      k_mutex_unlock(&rbuf_mutex);
    }
  }
}

K_THREAD_STACK_DEFINE(comm_stack_area, 1024);
struct k_thread comm_thread_data;

void comm_init(void (*on_signal)(const char* payload)) {
  if (!device_is_ready(uart_dev)) {
    // Can't report error via UART
    return;
  }

  // Configure UART interrupts
  uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
  uart_irq_rx_enable(uart_dev);

  // Command / signal process
  k_thread_create(&comm_thread_data, comm_stack_area,
                  K_THREAD_STACK_SIZEOF(comm_stack_area), comm_thread, NULL,
                  NULL, NULL, -1, K_FP_REGS,
                  K_NO_WAIT);  // lower than main thread(-1), cooperative thread

  // flush pre-init broken data (often bunch of zeros) on the serial line
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

void comm_ps_old_begin(const char* ps_type) {
  uart_write(ps_type, strlen(ps_type));
  uart_write(" ", 1);
}

void comm_ps_old_kv_str(const char* key, const char* fmt, ...) {
  uart_write(key, strlen(key));
  uart_write(":\"", 2);

  // value
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  // TODO: escape
  uart_puts(buffer);
  uart_write("\"", 1);
}

void comm_ps_old_kv_fmt(const char* key, const char* fmt, ...) {
  // key
  uart_write(key, strlen(key));

  // sep
  uart_write(":", 1);

  // value
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  uart_puts(buffer);
}

void comm_ps_old_kv_bool(const char* key, bool value) {
  uart_write(key, strlen(key));
  uart_write(":", 1);
  uart_puts(value ? "true" : "false");
}

void comm_ps_old_end() {
  uart_write("\n", 1);
}

void comm_print(const char* fmt, ...) {
  // Format and send message
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  uart_puts(buffer);
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

void comm_print_err(const char* fmt, ...) {
  uart_puts("err ");

  // Format and send message
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  uart_puts(buffer);
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

/**
 * (blocking) Wait until a command become available.
 */
void comm_wait_for_command() {
  while (1) {
    bool avail = false;
    k_mutex_lock(&rbuf_mutex, K_FOREVER);
    avail = recv_buffer_num > 0;
    k_mutex_unlock(&rbuf_mutex);

    if (avail) {
      return;
    }
    k_sleep(
        K_MSEC(10));  // very inefficient, prob good enough for interactive use
  }
}

/**
 * Get 1 command and peek next command, immediately.
 * Data will be copied to cmd, next_cmd.
 *
 * @returns number of available commands (0, 1, 2).
 */
int comm_get_command_if_avail(payload_t* cmd, payload_t* next_cmd) {
  int num;
  k_mutex_lock(&rbuf_mutex, K_FOREVER);
  if (recv_buffer_num > 0) {
    num = 1;
    memcpy(cmd, &recv_buffer, sizeof(payload_t));
    recv_buffer_num = 0;
  } else {
    num = 0;
  }
  k_mutex_unlock(&rbuf_mutex);
  return num;
}

void comm_get_next_command(char* buffer) {
  while (1) {
    // Wait for RX events
    uint32_t events =
        k_event_wait(&rx_events, RX_EVENT_COMMAND_RECEIVED | RX_EVENT_BACKSPACE,
                     false, K_FOREVER);

    if (events & RX_EVENT_BACKSPACE) {
      k_event_clear(&rx_events, RX_EVENT_BACKSPACE);

      uart_write((const uint8_t*)" \b", 2);  // backspace echo
      continue;
    }

    if (events & RX_EVENT_COMMAND_RECEIVED) {
      k_event_clear(&rx_events, RX_EVENT_COMMAND_RECEIVED);

      uart_write(LINE_ENDING, LINE_ENDING_LEN);  // echo new line

      // Trim leading whitespace
      char* trimmed = command_buffer;
      while (*trimmed == ' ' || *trimmed == '\t') {
        trimmed++;
      }

      // Only accept commands in IDLE state
      if (g_machine_state != STATE_IDLE) {
        continue;  // Silently ignore
      }

      // Copy command to caller's buffer
      strncpy(buffer, trimmed, 255);
      buffer[255] = '\0';
      return;
    }
  }
}

void comm_print_blob(uint8_t* ptr, int size) {
  uart_puts("blob ");

  char buffer[256];
  int pos = 0;

  const char* base64url_table =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

  // Encode in base64url (in chunks to fit buffer)
  for (int i = 0; i < size; i += 3) {
    // Flush buffer if getting full
    if (pos > sizeof(buffer) - 10) {
      uart_write((const uint8_t*)buffer, pos);
      pos = 0;
    }

    uint32_t val = 0;
    int chars = 0;

    // Pack up to 3 bytes into 24-bit value
    for (int j = 0; j < 3 && i + j < size; j++) {
      val = (val << 8) | ptr[i + j];
      chars++;
    }

    // Pad with zeros if needed
    val <<= (3 - chars) * 8;

    // Output base64url characters
    int output_chars = (chars == 1) ? 2 : (chars == 2) ? 3 : 4;
    buffer[pos++] = base64url_table[(val >> 18) & 0x3F];
    buffer[pos++] = base64url_table[(val >> 12) & 0x3F];
    if (output_chars > 2) {
      buffer[pos++] = base64url_table[(val >> 6) & 0x3F];
    }
    if (output_chars > 3) {
      buffer[pos++] = base64url_table[val & 0x3F];
    }
  }

  uart_write((const uint8_t*)buffer, pos);
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}
