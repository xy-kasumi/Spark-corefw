// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Semi-structured serial printing functions for the app.
 * As soon as serial is initialized, spark MUST use these functions.
 * Don't use printk(), LOG_*() etc.
 * Host of spark board depends on structured I/O.
 */
#pragma once

#include <stdint.h>

/** Initialize communication subsystem */
void comm_init();

/** Auto-prefix based on current machine state */
void comm_print(const char* fmt, ...);

/** Specific message types (override auto-prefix) */
void comm_print_ack();
void comm_print_err(const char* fmt, ...);
void comm_print_info(const char* fmt, ...);

/**
 * Get next command from console (blocking)
 * Buffer must be at least 256 bytes
 *
 * Note: "!" is NOT a regular command - it's handled immediately
 * and sets g_cancel_requested, then waits for the next command.
 * Commands are only accepted in IDLE state.
 */
void comm_get_next_command(char* buffer);

/**
 * Print binary data as base64url blob with checksum (single line)
 * Format: ">blob urlsafe-base64data... adler32hex"
 * Example: {1,2,3,4} outputs ">blob AQIDBA 0018000b"
 * @param ptr pointer to binary data
 * @param size number of bytes to print
 */
void comm_print_blob(uint8_t* ptr, int size);

// Future stream support
// void comm_print_stream_rem(uint32_t count);
// void comm_print_stream_seq(uint32_t seq, const char* fmt, ...);
