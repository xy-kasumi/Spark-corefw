// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "comm.h"

#include "comm_transport.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

////////////////////
// RX (down): commands (signals are immediately handled)

// big enough to hold 5 sec of commands
// 100 byte line takes 10ms at 100kbaud/s.
// NOTE: command that instantly finishes always starves the queue, though. it's
// up to the host to make commands "big" enough.
#define COMMAND_QUEUE_CAPACITY 500

K_MSGQ_DEFINE(cmd_msgq, sizeof(payload_t), COMMAND_QUEUE_CAPACITY, 1);

////////////////////
// TX (up): p-states (high & normal priority)

// chosen to not block any typical write burst, esp errors.
#define TX_QUEUE_PRIO_CAPACITY 25
#define TX_QUEUE_CAPACITY 100

K_MSGQ_DEFINE(tx_msgq_prio, sizeof(payload_t), TX_QUEUE_PRIO_CAPACITY, 1);
K_MSGQ_DEFINE(tx_msgq, sizeof(payload_t), TX_QUEUE_CAPACITY, 1);

static bool payload_is_signal(const payload_t* payload) {
  if (payload->size == 0) {
    return false;
  }
  char first_char = payload->data[0];
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
      tran_get_payload(&payload, K_NO_WAIT);

      if (payload_is_signal(&payload)) {
        on_signal(&payload);
      } else {
        k_msgq_put(&cmd_msgq, &payload, K_FOREVER);
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

  // Create prio 0 (preemptible, same as main) thread.
  k_thread_create(&comm_thread_data, comm_stack_area,
                  K_THREAD_STACK_SIZEOF(comm_stack_area), comm_thread,
                  (void*)on_signal, NULL, NULL, 0, K_FP_REGS, K_NO_WAIT);
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
  struct k_poll_event event;
  k_poll_event_init(&event, K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, &cmd_msgq);
  k_poll(&event, 1, K_FOREVER);
}

void comm_clear_commands() {
  k_msgq_purge(&cmd_msgq);
}

/**
 * Get 1 command and peek next command, immediately.
 * Data will be copied to cmd, next_cmd.
 *
 * @returns number of available commands (0, 1, 2).
 */
int comm_get_command_if_avail(payload_t* cmd, payload_t* next_cmd) {
  int num;
  k_sched_lock();
  int num_in_q = k_msgq_num_used_get(&cmd_msgq);
  if (num_in_q >= 2) {
    num = 2;
    k_msgq_peek_at(&cmd_msgq, next_cmd, 1);
    k_msgq_get(&cmd_msgq, cmd, K_NO_WAIT);
  } else if (num_in_q == 1) {
    num = 1;
    k_msgq_get(&cmd_msgq, cmd, K_NO_WAIT);
  } else {
    num = 0;
  }
  k_sched_unlock();
  return num;
}

void comm_stat_command_queue(int* num_cap, int* num_used) {
  if (num_cap) {
    *num_cap = COMMAND_QUEUE_CAPACITY;
  }
  if (num_used) {
    *num_used = k_msgq_num_used_get(&cmd_msgq);
  }
}
