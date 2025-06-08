// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Raw serial communcation.
 * 99% of the app code should use comm.h instead.
 * We need this (instead of using CONSOLE functionality),
 * because we wanted to disable console echo.
 */
#pragma once

#include <stdarg.h>

void comm_raw_init();
void comm_raw_putc(char c);
void comm_raw_puts(const char* str);
void comm_raw_printf(const char* fmt, ...);
void comm_raw_vprintf(const char* fmt, va_list args);
int comm_raw_poll_in(unsigned char* ch);
