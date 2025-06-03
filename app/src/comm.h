#pragma once

// Protocol-aware communication functions

// Initialize communication subsystem
void comm_init();

// Auto-prefix based on current machine state
void comm_print(const char* fmt, ...);

// Specific message types (override auto-prefix)
void comm_print_ack();
void comm_print_err(const char* fmt, ...);
void comm_print_info(const char* fmt, ...);

// Blocking read of one command from console
char* comm_read_command();

// Get next command from input thread (blocking)
// Buffer must be at least 256 bytes
void comm_get_next_command(char* buffer);

// Future stream support
// void comm_print_stream_rem(uint32_t count);
// void comm_print_stream_seq(uint32_t seq, const char* fmt, ...);