// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Settings system for spark core firmware.
 *
 * Settings use a "push" model:
 * - Settings are "pushed" to subsystems on settings_set().
 * - Subsystems should NEVER "pull" settings values directly.
 * - This ensures state consistency.
 */
#pragma once

#include <stdbool.h>

/** (blocking) Set a setting value. Returns true on success. */
bool settings_set(const char* key, float value);

/** Get a setting value. Returns 0.0f if key not found. */
float settings_get(const char* key);

/** Get setting by index. Returns false if index out of bounds. */
bool settings_get_by_index(int index, const char** key, float* value);

/** (blocking) Apply all current settings to hardware. */
void settings_apply_all();
