// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Semi-structured serial printing functions for the app.
 * As soon as serial is initialized, spark MUST use these functions.
 * Don't use printk(), LOG_*() etc., as the host of spark board depends on
 * structured I/O.
 *
 * All output lines are automatically prefixed by "I", ">" or "@" which
 * indicates the current state.
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

void comm_ps_old_begin(const char* ps_type);
void comm_ps_old_kv_str(const char* key, const char* fmt, ...);
void comm_ps_old_kv_fmt(const char* key, const char* fmt, ...);
void comm_ps_old_kv_bool(const char* key, bool value);
void comm_ps_old_end();

typedef enum {
  PS_POS,
  PS_QUEUE,
  PS_ERROR,
  PS_INIT,
  PS_SETTINGS,
  //
  PS_BLOB,  // should be separate?
} ps_type_t;

/** Partial state. */
typedef struct {
} pstate_t;

typedef struct {
  char data[100 + 6 + 1];
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

/** Get new partial state context. */
void comm_ps_begin(ps_type_t ps);

/** Add string value. */
void comm_ps_kv_str(ps_type_t ps, const char* key, const char* fmt, ...);

/** Add hex value. */
void comm_ps_kv_u32_hex(ps_type_t ps, const char* key, uint32_t value);

/** Add float value. */
void comm_ps_kv_float(ps_type_t ps, const char* key, float value);

/** Add boolean value. */
void comm_ps_kv_bool(ps_type_t ps, const char* key, bool value);

/** Finish the partial state. */
void comm_ps_end(ps_type_t ps);

/** (blocking) Print info message. */
void comm_print(const char* fmt, ...);

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
void comm_print_blob(uint8_t* ptr, int size);
