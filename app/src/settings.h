// Settings system for spark core firmware.

#pragma once

#include <stdbool.h>

/** Set a setting value. Returns true on success. */
bool settings_set(const char* key, float value);

/** Get a setting value. Returns 0.0f if key not found. */
float settings_get(const char* key);

/** Get setting by index. Returns false if index out of bounds. */
bool settings_get_by_index(int index, const char** key, float* value);

/** Apply all current settings to hardware. */
void settings_apply_all();
