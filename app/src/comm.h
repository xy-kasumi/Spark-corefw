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

#include <stdint.h>

/** Initialize communication subsystem */
void comm_init();

/** (blocking) Print generic informational string. */
void comm_print(const char* fmt, ...);

/** (blocking) Print for specific message types. */
void comm_print_ack();
void comm_print_err(const char* fmt, ...);
void comm_print_info(const char* fmt, ...);

/**
 * (blocking) Get next command from console.
 * Buffer must be at least 256 bytes
 *
 * Note 1: this will not return "!". Instead, it's processed internally
 * and sets g_cancel_requested, then waits for the next command.
 *
 * Note 2: Commands that come in non-IDLE states are silently ignored.
 */
void comm_get_next_command(char* buffer);

/**
 * (blocking) Print binary data as base64url blob with checksum as big single
 * line.
 *
 * Format: ">blob urlsafe-base64data... adler32hex"
 * Example: {1,2,3,4} outputs ">blob AQIDBA 0018000b"
 * @param ptr pointer to binary data
 * @param size number of bytes to print
 */
void comm_print_blob(uint8_t* ptr, int size);
