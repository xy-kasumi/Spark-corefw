// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) State machine which entire app depends on.
 */
#pragma once

#include <stdbool.h>

/** Cancel request flag (volatile for thread safety) */
extern volatile bool g_cancel_requested;

// State machine functions
void state_machine_init();
