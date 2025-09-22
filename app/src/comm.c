// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "comm.h"

#include "comm_transport.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

////////////////////
// RX (down)

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

////////////////////
// TX (up)

#define TX_QUEUE_PRIO_CAPACITY 25
#define TX_QUEUE_CAPACITY 100

K_MSGQ_DEFINE(tx_msgq_prio, sizeof(payload_t), TX_QUEUE_PRIO_CAPACITY, 1);
K_MSGQ_DEFINE(tx_msgq, sizeof(payload_t), TX_QUEUE_CAPACITY, 1);

static bool payload_is_signal(slice_t payload) {
  if (payload.size == 0) {
    return false;
  }
  char first_char = payload.ptr[0];
  return first_char == '!' || first_char == '?';
}

static void comm_thread(void* p1, void* p2, void* p3) {
  payload_handler_t on_signal = (payload_handler_t)p1;

  struct k_poll_event events[3];
  const int EV_IX_TX_PRIO = 0;
  const int EV_IX_TX = 1;
  const int EV_IX_RX = 2;
  k_poll_event_init(&events[EV_IX_TX_PRIO], K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, &tx_msgq_prio);
  k_poll_event_init(&events[EV_IX_TX], K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, &tx_msgq);
  tran_poll_event_get(&events[EV_IX_RX]);

  while (true) {
    if (k_poll(events, ARRAY_SIZE(events), K_FOREVER) != 0) {
      // (shouldn't happen) err on the side of communication.
      continue;
    }

    // Events matter in this order.
    // 1. TX_PRIO might contain critical error/debug information
    // 2. RX might contain cancel signal which nullifies further TX.
    // 3. TX is most likely to get stuck with infinite data bug.

    if (events[EV_IX_TX_PRIO].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
      payload_t payload;
      k_msgq_get(&tx_msgq_prio, &payload, K_NO_WAIT);
      tran_put_payload(&payload);

      events[EV_IX_TX_PRIO].state = K_POLL_STATE_NOT_READY;
    } else if (events[EV_IX_RX].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
      payload_t payload;
      tran_get_payload(&payload, K_FOREVER);

      slice_t line_slice = {payload.size, payload.data};
      if (payload_is_signal(line_slice)) {
        on_signal(line_slice);
      } else {
        // Copy command to caller's buffer
        k_mutex_lock(&rbuf_mutex, K_FOREVER);
        if (recv_buffer_num < RECV_BUFFER_CAPACITY) {
          recv_buffer[recv_buffer_ix_write] = payload;
          recv_buffer_ix_write =
              (recv_buffer_ix_write + 1) % RECV_BUFFER_CAPACITY;
          recv_buffer_num++;
        } else {
          // silently drop when buffer is full.
        }
        k_mutex_unlock(&rbuf_mutex);
      }

      events[EV_IX_RX].state = K_POLL_STATE_NOT_READY;
    } else if (events[EV_IX_TX].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
      payload_t payload;
      k_msgq_get(&tx_msgq, &payload, K_NO_WAIT);
      tran_put_payload(&payload);

      events[EV_IX_TX].state = K_POLL_STATE_NOT_READY;
    }
  }
}

K_THREAD_STACK_DEFINE(comm_stack_area, 1024);
struct k_thread comm_thread_data;

void comm_init(payload_handler_t on_signal) {
  if (!tran_init()) {
    // can't do any reporting
    return;
  }

  // Command / signal process
  k_thread_create(&comm_thread_data, comm_stack_area,
                  K_THREAD_STACK_SIZEOF(comm_stack_area), comm_thread,
                  (void*)on_signal, NULL, NULL, -1, K_FP_REGS,
                  K_NO_WAIT);  // lower than main thread(-1), cooperative thread
}

static int copy_str(uint8_t* buf, const char* str) {
  int len = strlen(str);
  memcpy(buf, str, len);
  return len;
}

static int copy_ps_tag(ps_type_t ps, uint8_t* buf) {
  const char* tag = "";  // not used, but to suppress warning
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
    case PS_EDM:
      tag = "edm ";
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
    case PS_STAT:
      tag = "stat ";
      break;
  }
  return copy_str(buf, tag);
}

static void put_payload(const uint8_t* ptr, int size, ps_type_t ps) {
  payload_t payload;
  payload.size = size;
  memcpy(payload.data, ptr, size);

  bool high_prio = (ps == PS_ERROR);
  if (high_prio) {
    k_msgq_put(&tx_msgq_prio, &payload, K_FOREVER);
  } else {
    k_msgq_put(&tx_msgq, &payload, K_FOREVER);
  }
}

void comm_ps_raw(ps_type_t ps, const char* fmt, ...) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  va_list args;
  va_start(args, fmt);
  offset += vsnprintf(buffer + offset, sizeof(buffer) - offset, fmt, args);
  va_end(args);

  put_payload(buffer, offset, ps);
}

void comm_ps_begin(ps_type_t ps) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  buffer[offset] = '<';
  offset++;

  put_payload(buffer, offset, ps);
}

void comm_ps_k_vfmtstr(ps_type_t ps, const char* key, const char* fmt, ...) {
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

  put_payload(buffer, offset, ps);
}

void comm_ps_k_v32hex(ps_type_t ps, const char* key, uint32_t value) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":");
  offset += snprintf(buffer + offset, sizeof(buffer) - offset, "0x%08x", value);

  put_payload(buffer, offset, ps);
}

void comm_ps_k_vfloat(ps_type_t ps, const char* key, float value) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":");
  offset +=
      snprintf(buffer + offset, sizeof(buffer) - offset, "%g", (double)value);

  put_payload(buffer, offset, ps);
}

void comm_ps_k_vint(ps_type_t ps, const char* key, int value) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":");
  offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%d", value);

  put_payload(buffer, offset, ps);
}

void comm_ps_k_vbool(ps_type_t ps, const char* key, bool value) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  offset += copy_str(buffer + offset, key);
  offset += copy_str(buffer + offset, ":");
  offset += copy_str(buffer + offset, value ? "true" : "false");

  put_payload(buffer, offset, ps);
}

void comm_ps_end(ps_type_t ps) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(ps, buffer);

  buffer[offset] = '>';
  offset++;

  put_payload(buffer, offset, ps);
}

void comm_error(slice_t source, const char* fmt, ...) {
  uint8_t buffer[PAYLOAD_BUFFER_SIZE];
  int offset = copy_ps_tag(PS_ERROR, buffer);

  offset += copy_str(buffer + offset, "<");

  if (!sl_is_empty(source)) {
    source = sl_sub(source, 0, 50);  // limit size
    offset += copy_str(buffer + offset, " src:");
    offset += copy_str(buffer + offset, "\"");
    memcpy(buffer + offset, source.ptr,
           source.size);  // TODO: size check & escape
    offset += source.size;
    offset += copy_str(buffer + offset, "\"");
  }

  // msg
  offset += copy_str(buffer + offset, " msg:");
  offset += copy_str(buffer + offset, "\"");
  // TODO: escape
  va_list args;
  va_start(args, fmt);
  offset += vsnprintf(buffer + offset, sizeof(buffer) - offset, fmt, args);
  va_end(args);

  offset += copy_str(buffer + offset, "\"");

  offset += copy_str(buffer + offset, " >");
  put_payload(buffer, offset, PS_ERROR);
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
    memcpy(cmd, &recv_buffer[recv_buffer_ix_read(0)], sizeof(payload_t));
    memcpy(next_cmd, &recv_buffer[recv_buffer_ix_read(1)], sizeof(payload_t));

    recv_buffer_num--;
  } else if (recv_buffer_num == 1) {
    num = 1;
    memcpy(cmd, &recv_buffer[recv_buffer_ix_read(0)], sizeof(payload_t));
    recv_buffer_num--;
  } else {
    num = 0;
  }
  k_mutex_unlock(&rbuf_mutex);
  return num;
}

void comm_stat_command_queue(int* num_cap, int* num_used) {
  k_mutex_lock(&rbuf_mutex, K_FOREVER);
  if (num_cap) {
    *num_cap = RECV_BUFFER_CAPACITY;
  }
  if (num_used) {
    *num_used = recv_buffer_num;
  }
  k_mutex_unlock(&rbuf_mutex);
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
