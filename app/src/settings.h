// Settings system for spark core firmware.

#pragma once

#include <stdbool.h>
#include <zephyr/device.h>

// External motor devices (defined in main.c)
extern const struct device* motor0;
extern const struct device* motor1;
extern const struct device* motor2;

/** Set a setting value from string. Returns true on success. */
bool settings_set(const char* key, const char* value_str);

/** Get a setting value. Returns 0.0f if key not found. */
float settings_get(const char* key);

/** Get setting by index. Returns false if index out of bounds. */
bool settings_get_by_index(int index, const char** key, float* value);

/** Apply all current settings to hardware. */
void settings_apply_all();
