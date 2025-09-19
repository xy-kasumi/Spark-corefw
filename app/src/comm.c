// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "comm.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

// UART device
static const struct device* uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

///// Transport layer

#define LINE_BUFFER_SIZE \
  107  // 100 (payload) + 1 (seq) + 4 (hash) + 1 (newline) + 1 (0-term)

// Line ending for output
static const uint8_t LINE_ENDING[] = "\r\n";
#define LINE_ENDING_LEN 2

// RX buffer and state
static char line_buffer[LINE_BUFFER_SIZE];
static volatile int rx_pos = 0;
static K_EVENT_DEFINE(rx_events);

// RX event definitions
#define RX_EVENT_PAYLOAD_RECEIVED BIT(0)
#define RX_EVENT_BACKSPACE BIT(1)

// TX buffer and state
static uint8_t tx_buffer[256];
static K_MUTEX_DEFINE(tx_mutex);
static volatile int tx_len = 0;
static volatile int tx_pos = 0;
static K_SEM_DEFINE(tx_done, 1, 1);

///// Application layer

// Recv buffer
#define RECV_BUFFER_CAPACITY 100
static K_MUTEX_DEFINE(rbuf_mutex);
payload_t recv_buffer[RECV_BUFFER_CAPACITY];
int recv_buffer_num = 0;       // number of commands
int recv_buffer_ix_write = 0;  // next write pos

// only valid when recv_buffer_num > 0 && offset < recv_buffer_num
// must be called within rbuf_mutex lock
inline int recv_buffer_ix_read(int offset) {
  return (recv_buffer_ix_write - recv_buffer_num + offset +
          RECV_BUFFER_CAPACITY) %
         RECV_BUFFER_CAPACITY;
}

// Tx buffer
typedef struct {
  int size;  // size w/o 0
  char data[PAYLOAD_BUFFER_SIZE];
} ps_buf_entry_t;

ps_buf_entry_t send_buffer[NUM_PS_TYPES];

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
        if (rx_pos > 0) {
          line_buffer[rx_pos] = '\0';
          rx_pos = 0;
          // TODO: what if previous RX_EVENT_PAYLOAD_RECEIVED was not proceeded
          // yet? (e.g. signal handler taking long)
          k_event_post(&rx_events, RX_EVENT_PAYLOAD_RECEIVED);
        }
      } else if (c == '\b' || c == 0x7F) {
        // Backspace: pop last char & notify to handle backspace echo
        if (rx_pos > 0) {
          rx_pos--;
          k_event_post(&rx_events, RX_EVENT_BACKSPACE);
        }
      } else if (c >= 0x20 && c <= 0x7E) {
        // Valid line content
        if (rx_pos < sizeof(line_buffer) - 1) {
          line_buffer[rx_pos] = c;
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
 * Write data. Caller is free to reuse data for other purposes.
 * This function will block until transmit is complete and internal buffer is
 * ready.
 *
 * @param data data to transmit
 * @param len Length of data (must be <= 256 bytes, excess will be silently
 * truncated)
 */
static void uart_write(const uint8_t* data, int len) {
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

static void comm_thread(void* p1, void* p2, void* p3) {
  payload_handler_t on_signal = (payload_handler_t)p1;

  while (1) {
    // Wait for RX events
    uint32_t events =
        k_event_wait(&rx_events, RX_EVENT_PAYLOAD_RECEIVED | RX_EVENT_BACKSPACE,
                     false, K_FOREVER);

    if (events & RX_EVENT_BACKSPACE) {
      k_event_clear(&rx_events, RX_EVENT_BACKSPACE);

      uart_write((const uint8_t*)" \b", 2);  // backspace echo
      continue;
    }

    if (events & RX_EVENT_PAYLOAD_RECEIVED) {
      k_event_clear(&rx_events, RX_EVENT_PAYLOAD_RECEIVED);

      uart_write(LINE_ENDING, LINE_ENDING_LEN);  // echo new line

      if (line_buffer[0] == '!' || line_buffer[0] == '?') {
        // signal
        on_signal(line_buffer);
      } else {
        // command
        // Copy command to caller's buffer
        k_mutex_lock(&rbuf_mutex, K_FOREVER);
        if (recv_buffer_num < RECV_BUFFER_CAPACITY) {
          strncpy(recv_buffer[recv_buffer_ix_write].data, line_buffer,
                  sizeof(payload_t));
          recv_buffer_ix_write++;
          recv_buffer_num++;
        } else {
          // silently drop when buffer is full.
        }
        k_mutex_unlock(&rbuf_mutex);
      }
    }
  }
}

K_THREAD_STACK_DEFINE(comm_stack_area, 1024);
struct k_thread comm_thread_data;

void comm_init(payload_handler_t on_signal) {
  if (!device_is_ready(uart_dev)) {
    // Can't report error via UART
    return;
  }

  // Configure UART interrupts
  uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
  uart_irq_rx_enable(uart_dev);

  // Command / signal process
  k_thread_create(&comm_thread_data, comm_stack_area,
                  K_THREAD_STACK_SIZEOF(comm_stack_area), comm_thread,
                  (void*)on_signal, NULL, NULL, -1, K_FP_REGS,
                  K_NO_WAIT);  // lower than main thread(-1), cooperative thread

  // flush pre-init broken data (often bunch of zeros) on the serial line
  uart_write(LINE_ENDING, LINE_ENDING_LEN);
}

static int copy_str(uint8_t* buf, const char* str) {
  int len = strlen(str);
  memcpy(buf, str, len);
  return len;
}

static int copy_ps_tag(ps_type_t ps, uint8_t* buf) {
  const char* tag;
  switch (ps) {
    case PS_ERROR:
      tag = "error ";
      break;
    case PS_POS:
      tag = "pos ";
      break;
    case PS_QUEUE:
      tag = "queue ";
      break;
    case PS_INIT:
      tag = "init ";
      break;
    case PS_SETTINGS:
      tag = "stg ";
      break;
    case PS_BLOB:
      tag = "blob ";
      break;
    default:
      // bug!
      return 0;
  }
  return copy_str(buf, tag);
}

void comm_ps_raw(ps_type_t ps, const char* fmt, ...) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  va_list args;
  va_start(args, fmt);
  offset += vsnprintf(buffer + offset, sizeof(buffer) - offset, fmt, args);
  va_end(args);

  uart_write(buffer, offset);
}

void comm_ps_begin(ps_type_t ps) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  buffer[offset] = '<';
  offset++;

  uart_write(buffer, offset);
}

void comm_ps_kv_str(ps_type_t ps, const char* key, const char* fmt, ...) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":\"");

  // value
  // TODO: escape
  va_list args;
  va_start(args, fmt);
  offset += vsnprintf(buffer + offset, sizeof(buffer) - offset, fmt, args);
  va_end(args);

  offset += copy_str(buffer + offset, "\"");

  uart_write(buffer, offset);
}

void comm_ps_kv_u32_hex(ps_type_t ps, const char* key, uint32_t value) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":");
  offset += snprintf(buffer + offset, sizeof(buffer) - offset, "0x%08x", value);

  uart_write(buffer, offset);
}

void comm_ps_kv_float(ps_type_t ps, const char* key, float value) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":");
  offset +=
      snprintf(buffer + offset, sizeof(buffer) - offset, "%g", (double)value);

  uart_write(buffer, offset);
}

void comm_ps_kv_bool(ps_type_t ps, const char* key, bool value) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":");
  offset += copy_str(buffer + offset, value ? "true" : "false");

  uart_write(buffer, offset);
}

void comm_ps_end(ps_type_t ps) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  buffer[offset] = '>';
  offset++;

  uart_write(buffer, offset);
}

void comm_print_err(const char* fmt, ...) {
  // GONE
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
    // busy waiting is very inefficient, but probably good enough for
    // interactive use
    k_sleep(K_MSEC(10));
  }
}

void comm_clear_commands() {
  k_mutex_lock(&rbuf_mutex, K_FOREVER);
  recv_buffer_num = 0;
  k_mutex_unlock(&rbuf_mutex);
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
  if (recv_buffer_num >= 2) {
    num = 2;
    strncpy(cmd->data, recv_buffer[recv_buffer_ix_read(0)].data,
            sizeof(payload_t));
    strncpy(next_cmd->data, recv_buffer[recv_buffer_ix_read(1)].data,
            sizeof(payload_t));
    recv_buffer_num--;
  } else if (recv_buffer_num == 1) {
    num = 1;
    strncpy(cmd->data, recv_buffer[recv_buffer_ix_read(0)].data,
            sizeof(payload_t));
    recv_buffer_num--;
  } else {
    num = 0;
  }
  k_mutex_unlock(&rbuf_mutex);
  return num;
}

/**
 * Convert data (ptr, size) to urlsafe-base64 and store in dst (no 0-term)
 * Returns number of bytes written to dst.
 *
 * Base 64 size = 4 * ceil(size / 3)
 *
 * Caller must ensure dst is big enough to store conerted data.
 */
static int copy_base64(uint8_t* dst, const uint8_t* src, int src_size) {
  const char* base64url_table =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  int dst_ofs = 0;

  // Encode in base64url (in chunks to fit buffer)
  for (int src_ofs = 0; src_ofs < src_size; src_ofs += 3) {
    // Pack 3 byte to 24-bit value with 0-padding
    uint32_t val = 0;
    int num_chars = 0;
    for (int i = 0; i < 3; i++) {
      val <<= 8;
      if (src_ofs + i < src_size) {
        val |= src[src_ofs + i];
        num_chars++;
      }
    }

    // Output base64url characters
    dst[dst_ofs++] = base64url_table[(val >> 18) & 0x3F];
    dst[dst_ofs++] = base64url_table[(val >> 12) & 0x3F];
    if (num_chars >= 2) {
      dst[dst_ofs++] = base64url_table[(val >> 6) & 0x3F];
    }
    if (num_chars >= 3) {
      dst[dst_ofs++] = base64url_table[val & 0x3F];
    }
  }
  return dst_ofs;
}

void comm_print_blob(const uint8_t* ptr, int size) {
  // "blob 0:...." total size = 100 byte
  // * fixed part ("blob ", ":"): 6 byte
  // * key (up to 999,999): 6 byte max
  // -> post-base64 data: 88 byte
  // -> pre-base64 data: 66 byte
  // blob can send max of 1M x 66B = 66MB. (big enough)
  const int orig_byte_per_msg = 66;

  int msg_ix = 0;
  int offset = 0;
  comm_ps_begin(PS_BLOB);
  while (offset < size) {
    int orig_bytes = orig_byte_per_msg;
    if (offset + orig_bytes > size) {
      orig_bytes = size - offset;
    }

    char b64buf[PAYLOAD_BUFFER_SIZE];
    int b64size = copy_base64((uint8_t*)b64buf, ptr + offset, orig_bytes);
    b64buf[b64size] = '\0';  // for fmt in comm_ps_raw
    offset += orig_bytes;

    comm_ps_raw(PS_BLOB, "%d:%s", msg_ix, b64buf);
    msg_ix++;
  }
  comm_ps_end(PS_BLOB);
}
