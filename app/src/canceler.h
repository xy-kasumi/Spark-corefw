// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) State machine whose whole purpose is to control entire app's
 * cancellation.
 */
#pragma once

#include <stdbool.h>

// Cancel is active for CANCEL_TIMEOUT, then reset to normal automatically,
// after canceler_cancel() is called. All modules must poll
// canceler_cancel_needed() frequently enough to be able to cancel their
// operation within this window.
#define CANCEL_TIMEOUT K_MSEC(500)

/**
 * (blocking) Initialize canceler module.
 */
void canceler_init();

/**
 * Invoke cancellation.
 */
void canceler_cancel();

/**
 * Returns true if caller should cancel current operation.
 */
bool canceler_cancel_needed();
