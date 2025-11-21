// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Pump control
 */
#pragma once

#include <stdbool.h>

/** Initialize pump GPIO. */
bool pump_init();

/** Set pump enable state (blocking).
 * @param enable true to start pump, false to stop pump
 */
void pump_set_enable(bool enable);
