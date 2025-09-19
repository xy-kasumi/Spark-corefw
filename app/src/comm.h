// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Semi-structured serial printing functions for the app.
 * Don't use printk(), LOG_*() etc. which will break the protocol.
 *
 * See spec/protocol.md for details.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Handles borrowed const payload.
typedef void (*payload_handler_t)(const char* payload);

/**
 * (blocking) Initialize communication subsystem.
 * Internally spawns long-running thread dedicated to communication.
 *
 * @param on_signal signal handler. Called from communication thread. Must
 * finish within 10ms. (faster is better)
 */
void comm_init(payload_handler_t on_signal);

// Sent in this order (lower number = higher prio)
// Must be densely packed as [0, NUM_PS_TYPES).
typedef enum {
  /** (automatic @ comm thread) */
  PS_ERROR = 0,

  /** (signal result @ comm thread) */
  PS_QUEUE = 1,

  /** (signal result @ comm thread) */
  PS_POS = 2,

  /** (automatic @ main thread) */
  PS_INIT = 3,

  /** (@ main thread) */
  PS_SETTINGS = 4,

  /** (@main thread) */
  PS_STAT = 5,

  /** (@ main thread) */
  PS_BLOB = 6,  // should be separate?
} ps_type_t;

#define NUM_PS_TYPES 7

/** Partial state. */
typedef struct {
} pstate_t;

#define PAYLOAD_BUFFER_SIZE 101  // 100 (payload) + 1 (0-term)

typedef struct {
  char data[PAYLOAD_BUFFER_SIZE];
} payload_t;

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

/** (blocking) Print error message. */
void comm_print_err(const char* fmt, ...);

/**
 * (blocking) Print blob as base64 with checksum in one big line.
 *
 * Format: ">blob urlsafe-base64data... adler32hex"
 * Example: {1,2,3,4} outputs ">blob AQIDBA 0018000b"
 * @param ptr pointer to binary data
 * @param size number of bytes to print
 */
void comm_print_blob(const uint8_t* ptr, int size);
