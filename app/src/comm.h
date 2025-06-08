/**
 * (Singleton) Semi-structured serial printing functions for the app.
 * As soon as serial is initialized, spark MUST use these functions.
 * Don't use printk(), LOG_*() etc.
 * Host of spark board depends on structured I/O.
 */
#pragma once

/** Initialize communication subsystem */
void comm_init();

/** Auto-prefix based on current machine state */
void comm_print(const char* fmt, ...);

/** Specific message types (override auto-prefix) */
void comm_print_ack();
void comm_print_err(const char* fmt, ...);
void comm_print_info(const char* fmt, ...);

/** Blocking read of one command from console */
char* comm_read_command();

/**
 * Get next command from input thread (blocking)
 * Buffer must be at least 256 bytes
 *
 * Note: "!" is NOT a regular command - it's handled asynchronously
 * by the input thread and sets g_cancel_requested immediately,
 * bypassing the normal command queue.
 */
void comm_get_next_command(char* buffer);

// Future stream support
// void comm_print_stream_rem(uint32_t count);
// void comm_print_stream_seq(uint32_t seq, const char* fmt, ...);
