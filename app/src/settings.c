#include "settings.h"

#include "motion.h"
#include "motor.h"
#include "strutil.h"

#include <drivers/tmc_driver.h>

#include <stdlib.h>
#include <string.h>

// Simple key->value dictionary
typedef struct {
  const char* key;
  float value;
} setting_entry_t;

// Settings array with all 3 motors and axes
static setting_entry_t settings[] = {
    // Motor settings
    {"m.0.microstep", 32.0f},
    {"m.0.current", 30.0f},
    {"m.0.thresh", 2.0f},
    {"m.0.unitsteps", 200.0f},
    {"m.1.microstep", 32.0f},
    {"m.1.current", 30.0f},
    {"m.1.thresh", 2.0f},
    {"m.1.unitsteps", -200.0f},
    {"m.2.microstep", 32.0f},
    {"m.2.current", 30.0f},
    {"m.2.thresh", 2.0f},
    {"m.2.unitsteps", -200.0f},
    // Axis settings
    {"a.x.origin", 0.0f},
    {"a.x.side", 1.0f},
    {"a.y.origin", 0.0f},
    {"a.y.side", -1.0f},
    {"a.z.origin", 0.0f},
    {"a.z.side", 1.0f},
};

#define SETTINGS_COUNT (sizeof(settings) / sizeof(settings[0]))

// Dictionary operations
static int find_setting_index(const char* key) {
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    if (strcmp(settings[i].key, key) == 0) {
      return i;
    }
  }
  return -1;
}

// Motor-specific setting application under "m."
static bool apply_motor(char* mut_key, float value) {
  // Parse: {motor_num}.{key}
  char* rest = split_at(mut_key, '.');
  if (!rest) {
    // invalid key format.
    return false;
  }

  // Get motor device
  int motor_num;
  if (!parse_int(mut_key, &motor_num)) {
    return false;  // Invalid motor number
  }

  const struct device* motor = motor_get_device(motor_num);
  if (!motor) {
    return false;  // Invalid motor number
  }

  // Apply setting
  int ret = 0;
  if (strcmp(rest, "microstep") == 0) {
    ret = tmc_set_microstep(motor, (int)value);
  } else if (strcmp(rest, "current") == 0) {
    ret = tmc_set_current(motor, (int)value, 0);
  } else if (strcmp(rest, "thresh") == 0) {
    ret = tmc_set_stallguard_threshold(motor, (int)value);
  } else if (strcmp(rest, "unitsteps") == 0) {
    motion_set_motor_unitsteps(motor_num, value);
    ret = 0;  // Always succeeds
  } else {
    return false;
  }

  return ret == 0;
}

// Axis-specific setting application under "a."
static bool apply_axis(char* mut_key, float value) {
  // Parse: {axis_name}.{key}
  char* rest = split_at(mut_key, '.');
  if (!rest) {
    return false;  // invalid key format
  }

  // Get axis number from name
  int axis_num = -1;
  if (strcmp(mut_key, "x") == 0) {
    axis_num = 0;
  } else if (strcmp(mut_key, "y") == 0) {
    axis_num = 1;
  } else if (strcmp(mut_key, "z") == 0) {
    axis_num = 2;
  } else {
    return false;  // Invalid axis name
  }

  // Apply setting
  if (strcmp(rest, "origin") == 0) {
    motion_set_home_origin(axis_num, value);
    return true;
  } else if (strcmp(rest, "side") == 0) {
    motion_set_home_side(axis_num, value);
    return true;
  }

  return false;
}

// Hierarchical apply dispatcher
static bool apply_setting(const char* key, float value) {
  // Make mutable copy for parsing
  char mut_key[64];
  strncpy(mut_key, key, sizeof(mut_key) - 1);
  mut_key[sizeof(mut_key) - 1] = '\0';

  char* rest = split_at(mut_key, '.');
  if (!rest) {
    // invalid key format.
    return false;
  }

  if (strcmp(mut_key, "m") == 0) {
    return apply_motor(rest, value);
  } else if (strcmp(mut_key, "a") == 0) {
    return apply_axis(rest, value);
  }
  return false;
}

// Public API
bool settings_set(const char* key, float value) {
  // Check if key exists
  int index = find_setting_index(key);
  if (index < 0) {
    return false;  // Key not found
  }

  // Try to apply
  if (apply_setting(key, value)) {
    // Update dict on success
    settings[index].value = value;
    return true;
  }
  return false;  // Apply failed
}

float settings_get(const char* key) {
  int index = find_setting_index(key);
  return (index >= 0) ? settings[index].value : 0.0f;
}

bool settings_get_by_index(int index, const char** key, float* value) {
  if (index < 0 || index >= SETTINGS_COUNT) {
    return false;
  }
  if (key) {
    *key = settings[index].key;
  }
  if (value) {
    *value = settings[index].value;
  }
  return true;
}

void settings_apply_all() {
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    (void)apply_setting(settings[i].key, settings[i].value);
  }
}
