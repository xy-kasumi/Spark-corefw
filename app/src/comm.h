// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Semi-structured serial printing functions for the app.
 * Don't use printk(), LOG_*() etc. which will break the protocol.
 *
 * See spec/protocol.md for details.
 */
#pragma once

#include "comm_transport.h"
#include "slice.h"

#include <stdbool.h>
#include <stdint.h>

// Handles borrowed const payload.
typedef void (*payload_handler_t)(payload_t* payload);

/**
 * (blocking) Initialize communication subsystem.
 * Internally spawns long-running thread dedicated to communication.
 *
 * @param on_signal signal handler. Called from communication thread. Must
 * finish within 10ms. (faster is better)
 */
void comm_init(payload_handler_t on_signal);

// P-state types.
// PS_ERROR is special, as it's sent in high-priority queue.
typedef enum {
  PS_ERROR,
  PS_QUEUE,
  PS_POS,
  PS_EDM,
  PS_INIT,
  PS_SETTINGS,
  PS_STAT,
  PS_BLOB,
} ps_type_t;

/**
 * (blocking) Wait until a command become available.
 */
void comm_wait_for_command();

/**
 * Get 1 command and peek next command, immediately.
 * Data will be copied to cmd, next_cmd.
 *
 * @returns number of available commands (0, 1, 2).
 */
int comm_get_command_if_avail(payload_t* cmd, payload_t* next_cmd);

/**
 * Clears command buffer.
 * OK to call from different thread simultaneously while comm_wait_for_command()
 * or comm_get_command_if_avail() is running.
 */
void comm_clear_commands();

/**
 * Fetches current command queue statistics.
 * @param num_cap (optional) capacity of command queue will be stored.
 * @param num_used (optional) number of used slots in command queue will be
 * stored.
 */
void comm_stat_command_queue(int* num_cap, int* num_used);

/**
 * Print entire p-state.
 * e.g. fmt=="< x:%f y:%f >"
 * Caller must guarantee entire thing (including tag for ps) fits in
 * PAYLOAD_BUFFER_SIZE.
 */
void comm_ps_raw(ps_type_t ps, const char* fmt, ...);

/** Get new partial state context. */
void comm_ps_begin(ps_type_t ps);

/** Add string value. */
void comm_ps_k_vfmtstr(ps_type_t ps, const char* key, const char* fmt, ...);

/** Add hex value. */
void comm_ps_k_v32hex(ps_type_t ps, const char* key, uint32_t value);

/** Add float value. */
void comm_ps_k_vfloat(ps_type_t ps, const char* key, float value);

/** Add int value. */
void comm_ps_k_vint(ps_type_t ps, const char* key, int value);

/** Add boolean value. */
void comm_ps_k_vbool(ps_type_t ps, const char* key, bool value);

/** Finish the partial state. */
void comm_ps_end(ps_type_t ps);

/** Print error message. */
void comm_error(slice_t source, const char* fmt, ...);

/** Assert that cond is true, and print error otherwise. Non-blocking unless too many errors occur at once. */
#define comm_assert(cond)                                                    \
  {                                                                          \
    if (!(cond)) {                                                           \
      char* fname = strrchr(__FILE__, '/');                                  \
      fname = fname == NULL ? __FILE__ : (fname + 1);                        \
      comm_ps_raw(PS_ERROR, "< msg:\"assert failed\" file:\"%s\" line:%d >", \
                  fname, __LINE__);                                          \
    }                                                                        \
  }
