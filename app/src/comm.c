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
static K_SEM_DEFINE(rx_sem, 0, 1);

// TX buffer and state
static uint8_t tx_buffer[256];
static K_MUTEX_DEFINE(tx_mutex);
static volatile int tx_len = 0;
static volatile int tx_pos = 0;
static K_SEM_DEFINE(tx_done, 1, 1);

// UART interrupt handler
static void uart_isr(const struct device* dev, void* user_data) {
  uart_irq_update(dev);

  // RX handling - directly into command_buffer
  if (uart_irq_rx_ready(dev)) {
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
      if (c == '\r' || c == '\n') {
        // Can accept any of CR, CRLF, LF.
        if (rx_pos > 0) {
          command_buffer[rx_pos] = '\0';
          rx_pos = 0;
          k_sem_give(&rx_sem);  // Signal command ready
        }
      } else if (c == '\b' || c == 0x7F) {
        // Backspace handling (terminal handles echo)
        if (rx_pos > 0) {
          rx_pos--;
        }
      } else if (c >= 0x20 && c <= 0x7E) {
        // Printable character
        if (rx_pos < sizeof(command_buffer) - 1) {
          command_buffer[rx_pos++] = c;
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
      tx_pos = 0;
      tx_len = 0;
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

// Helper to write string
static void uart_puts(const char* str) {
  uart_write((const uint8_t*)str, strlen(str));
}

void comm_init() {
  if (!device_is_ready(uart_dev)) {
    // Can't report error via UART
    return;
  }

  // Configure UART interrupts
  uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
  uart_irq_rx_enable(uart_dev);
}

void comm_print(const char* fmt, ...) {
  char buffer[256];
  char* ptr = buffer;

  // Add prefix based on state
  switch (state_machine_get_state()) {
    case STATE_IDLE:
      strcpy(ptr, "I ");
      ptr += 2;
      break;
    case STATE_EXEC_INTERACTIVE:
      strcpy(ptr, "> ");
      ptr += 2;
      break;
    case STATE_EXEC_STREAM:
      strcpy(ptr, "@ ");
      ptr += 2;
      break;
  }

  // Format message
  va_list args;
  va_start(args, fmt);
  vsnprintf(ptr, buffer + sizeof(buffer) - ptr, fmt, args);
  va_end(args);

  uart_puts(buffer);
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

void comm_print_ack() {
  uart_puts(">ack");
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

void comm_print_err(const char* fmt, ...) {
  char buffer[256];
  char* ptr = buffer;
  machine_state_t state = state_machine_get_state();

  // Choose prefix based on state
  if (state == STATE_EXEC_INTERACTIVE) {
    strcpy(ptr, ">err ");
    ptr += 5;
  } else if (state == STATE_EXEC_STREAM) {
    strcpy(ptr, "@err ");
    ptr += 5;
  } else {
    strcpy(ptr, "I ");
    ptr += 2;
  }

  // Format message
  va_list args;
  va_start(args, fmt);
  vsnprintf(ptr, buffer + sizeof(buffer) - ptr, fmt, args);
  va_end(args);

  uart_puts(buffer);
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

void comm_print_info(const char* fmt, ...) {
  char buffer[256];
  char* ptr = buffer;
  machine_state_t state = state_machine_get_state();

  // Choose prefix based on state
  if (state == STATE_EXEC_INTERACTIVE) {
    strcpy(ptr, ">inf ");
    ptr += 5;
  } else if (state == STATE_EXEC_STREAM) {
    strcpy(ptr, "@inf ");
    ptr += 5;
  } else {
    strcpy(ptr, "I ");
    ptr += 2;
  }

  // Format message
  va_list args;
  va_start(args, fmt);
  vsnprintf(ptr, buffer + sizeof(buffer) - ptr, fmt, args);
  va_end(args);

  uart_puts(buffer);
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

void comm_get_next_command(char* buffer) {
  while (1) {
    // Wait for command from ISR
    k_sem_take(&rx_sem, K_FOREVER);

    // Echo newline
    uart_write(LINE_ENDING, LINE_ENDING_LEN);

    // Trim leading whitespace
    char* trimmed = command_buffer;
    while (*trimmed == ' ' || *trimmed == '\t') {
      trimmed++;
    }

    // Special handling for "!" - always process immediately
    if (strcmp(trimmed, "!") == 0) {
      g_cancel_requested = true;
      continue;  // Wait for next command
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

// Adler-32 checksum calculation
static uint32_t adler32(const uint8_t* data, int len) {
  uint32_t a = 1, b = 0;
  for (int i = 0; i < len; i++) {
    a = (a + data[i]) % 65521;
    b = (b + a) % 65521;
  }
  return (b << 16) | a;
}

void comm_print_blob(uint8_t* ptr, int size) {
  char buffer[256];
  int pos = 0;

  // Start with prefix
  strcpy(buffer, ">blob ");
  pos = 6;

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

  // Add checksum
  buffer[pos++] = ' ';

  uint32_t checksum = adler32(ptr, size);
  const char* hex_table = "0123456789abcdef";
  for (int i = 7; i >= 0; i--) {
    buffer[pos++] = hex_table[(checksum >> (i * 4)) & 0x0F];
  }

  uart_write((const uint8_t*)buffer, pos);
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}
