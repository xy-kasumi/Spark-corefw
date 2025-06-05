#include "settings.h"

#include <drivers/tmc_driver.h>

#include <stdlib.h>
#include <string.h>

// Simple key->value dictionary
typedef struct {
  const char* key;
  float value;
} setting_entry_t;

// Settings array with current cmd_set supported keys
static setting_entry_t settings[] = {
    {"mot0.microstep", 32.0f},
    {"mot0.current", 30.0f},
    {"mot0.thresh", 2.0f},
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

// Apply function - direct port of cmd_set logic
static bool apply_setting(const char* key, float value) {
  int ret = 0;

  if (strcmp(key, "mot0.microstep") == 0) {
    ret = tmc_set_microstep(motor0, (int)value);
  } else if (strcmp(key, "mot0.current") == 0) {
    ret = tmc_set_current(motor0, (int)value, 0);
  } else if (strcmp(key, "mot0.thresh") == 0) {
    ret = tmc_set_stallguard_threshold(motor0, (int)value);
  } else {
    return false;
  }

  return ret == 0;
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
