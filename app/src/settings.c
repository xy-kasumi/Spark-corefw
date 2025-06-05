#include "settings.h"
#include "strutil.h"

#include <drivers/tmc_driver.h>

#include <stdlib.h>
#include <string.h>

// Simple key->value dictionary
typedef struct {
  const char* key;
  float value;
} setting_entry_t;

// Settings array with all 3 motors
static setting_entry_t settings[] = {
    {"m.0.microstep", 32.0f}, {"m.0.current", 30.0f}, {"m.0.thresh", 2.0f},
    {"m.1.microstep", 32.0f}, {"m.1.current", 30.0f}, {"m.1.thresh", 2.0f},
    {"m.2.microstep", 32.0f}, {"m.2.current", 30.0f}, {"m.2.thresh", 2.0f},
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

  const struct device* motor;
  switch (motor_num) {
    case 0:
      motor = motor0;
      break;
    case 1:
      motor = motor1;
      break;
    case 2:
      motor = motor2;
      break;
    default:
      return false;
  }

  // Apply setting
  int ret = 0;
  if (strcmp(rest, "microstep") == 0) {
    ret = tmc_set_microstep(motor, (int)value);
  } else if (strcmp(rest, "current") == 0) {
    ret = tmc_set_current(motor, (int)value, 0);
  } else if (strcmp(rest, "thresh") == 0) {
    ret = tmc_set_stallguard_threshold(motor, (int)value);
  } else {
    return false;
  }

  return ret == 0;
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
