#pragma once

#include <stdarg.h>

void comm_raw_init();
void comm_raw_putc(char c);
void comm_raw_puts(const char* str);
void comm_raw_printf(const char* fmt, ...);
void comm_raw_vprintf(const char* fmt, va_list args);
