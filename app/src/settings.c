// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "settings.h"

#include "comm.h"
#include "coords.h"
#include "gcode.h"
#include "motion.h"
#include "motor.h"
#include "strutil.h"
#include "wirefeed.h"

#include <drivers/tmc_driver.h>

#include <stdlib.h>
#include <string.h>

// Simple key->value dictionary
typedef struct {
  const char* key;
  float value;
} setting_entry_t;

// Settings array with all 3 motors and axes (sorted by key)
// TODO: Move machine-specific config to spooler.
static setting_entry_t settings[] = {
    // Axis settings
    {"a.x.home.origin", 0.0f},
    {"a.x.home.phase", 1.0f},
    {"a.x.home.side", 1.0f},
    {"a.y.home.origin", 0.0f},
    {"a.y.home.phase", 2.0f},
    {"a.y.home.side", -1.0f},
    {"a.z.home.origin", 0.0f},
    {"a.z.home.phase", 0.0f},
    {"a.z.home.side", 1.0f},
    // Coordinate system settings
    {"cs.g.pos.x", -55.f},
    {"cs.g.pos.y", 78.f},
    {"cs.g.pos.z", -84.5f},
    {"cs.w.pos.x", -58.f},
    {"cs.w.pos.y", 17.f},
    {"cs.w.pos.z", -89.f},
    // Motor settings
    {"m.0.current", 30.0f},
    {"m.0.idlems", 200.0f},
    {"m.0.microstep", 32.0f},
    {"m.0.thresh", 2.0f},
    {"m.0.unitsteps", 200.0f},
    {"m.1.current", 30.0f},
    {"m.1.idlems", 200.0f},
    {"m.1.microstep", 32.0f},
    {"m.1.thresh", 2.0f},
    {"m.1.unitsteps", -200.0f},
    {"m.2.current", 30.0f},
    {"m.2.idlems", 200.0f},
    {"m.2.microstep", 32.0f},
    {"m.2.thresh", 2.0f},
    {"m.2.unitsteps", -200.0f},
    {"m.3.current", 30.0f},
    {"m.3.idlems", 200.0f},
    {"m.3.microstep", 32.0f},
    {"m.3.thresh", 2.0f},
    {"m.3.unitsteps", 200.0f},
    {"m.4.current", 30.0f},
    {"m.4.idlems", 200.0f},
    {"m.4.microstep", 32.0f},
    {"m.4.thresh", 2.0f},
    {"m.4.unitsteps", 200.0f},
    {"m.5.current", 30.0f},
    {"m.5.idlems", 200.0f},
    {"m.5.microstep", 32.0f},
    {"m.5.thresh", 2.0f},
    {"m.5.unitsteps", 6400.0f},
    {"m.6.current", 30.0f},
    {"m.6.idlems", 5000.0f},
    {"m.6.microstep", 32.0f},
    {"m.6.thresh", 2.0f},
    {"m.6.unitsteps", 203.8f},
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
    // if (motor_num == 6) {
    // same hold current as move current
    ret = tmc_set_current(motor, (int)value, (int)value);
    //} else {
    //      ret = tmc_set_current(motor, (int)value, 0);
    //  }
  } else if (strcmp(rest, "thresh") == 0) {
    ret = tmc_set_stallguard_threshold(motor, (int)value);
  } else if (strcmp(rest, "unitsteps") == 0) {
    motion_set_motor_unitsteps(motor_num, value);
    // Also update wirefeed if this is motor6
    if (motor_num == 6) {
      wirefeed_set_unitsteps(value);
    }
    ret = 0;  // Always succeeds
  } else if (strcmp(rest, "idlems") == 0) {
    motor_deenergize_after(motor_num, (int)value);
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
  axis_t axis_num;
  if (strcmp(mut_key, "x") == 0) {
    axis_num = AXIS_X;
  } else if (strcmp(mut_key, "y") == 0) {
    axis_num = AXIS_Y;
  } else if (strcmp(mut_key, "z") == 0) {
    axis_num = AXIS_Z;
  } else {
    return false;  // Invalid axis name
  }

  // Parse {subsystem}.{property} (e.g., "home.origin")
  char* property = split_at(rest, '.');
  if (!property) {
    return false;
  }

  if (strcmp(rest, "home") != 0) {
    return false;  // Must be "home"
  }

  // Apply setting
  if (strcmp(property, "origin") == 0) {
    motion_set_home_origin(axis_num, value);
    return true;
  } else if (strcmp(property, "side") == 0) {
    motion_set_home_side(axis_num, value);
    return true;
  } else if (strcmp(property, "phase") == 0) {
    gcode_set_home_phase(axis_num, (int)value);
    return true;
  }

  return false;
}

// Coordinate system-specific setting application under "cs."
static bool apply_cs(char* mut_key, float value) {
  // Parse: {coord_system}.pos.{axis}
  char* rest = split_at(mut_key, '.');
  if (!rest) {
    return false;
  }

  // Parse coordinate system and axis
  char* axis = split_at(rest, '.');
  if (!axis) {
    return false;
  }

  if (strcmp(rest, "pos") != 0) {
    return false;  // Must be "pos"
  }

  // Apply to appropriate coordinate system
  coord_system_t cs_type;
  axis_t axis_index;

  if (strcmp(mut_key, "g") == 0) {
    cs_type = COORD_SYSTEM_GRINDER;
  } else if (strcmp(mut_key, "w") == 0) {
    cs_type = COORD_SYSTEM_WORK;
  } else {
    return false;
  }

  if (strcmp(axis, "x") == 0) {
    axis_index = AXIS_X;
  } else if (strcmp(axis, "y") == 0) {
    axis_index = AXIS_Y;
  } else if (strcmp(axis, "z") == 0) {
    axis_index = AXIS_Z;
  } else if (strcmp(axis, "c") == 0) {
    axis_index = AXIS_C;
  } else {
    return false;
  }

  gcode_set_coord_offset(cs_type, axis_index, value);
  return true;
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
  } else if (strcmp(mut_key, "cs") == 0) {
    return apply_cs(rest, value);
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
  bool success = true;
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    bool res = apply_setting(settings[i].key, settings[i].value);
    if (!res) {
      comm_print_err("Failed to apply setting %s", settings[i].key);
      success = false;
    }
  }
  if (success) {
    comm_print("settings: init ok");
  } else {
    comm_print_err("settings: some settings broken (firmware bug)");
  }
}
