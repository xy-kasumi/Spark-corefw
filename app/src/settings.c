// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "settings.h"

#include "comm.h"
#include "coords.h"
#include "gcode.h"
#include "motion.h"
#include "motor.h"
#include "strutil.h"
#include "toolsupply.h"
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
    {"a.x.home.travel", 500.0f},
    {"a.y.home.origin", 0.0f},
    {"a.y.home.phase", 2.0f},
    {"a.y.home.side", -1.0f},
    {"a.y.home.travel", 500.0f},
    {"a.z.home.origin", 0.0f},
    {"a.z.home.phase", 0.0f},
    {"a.z.home.side", 1.0f},
    {"a.z.home.travel", 500.0f},
    // Coordinate system settings
    {"cs.g.pos.x", 0.0f},
    {"cs.g.pos.y", 0.0f},
    {"cs.g.pos.z", 0.0f},
    {"cs.w.pos.x", 0.0f},
    {"cs.w.pos.y", 0.0f},
    {"cs.w.pos.z", 0.0f},
    {"cs.ts.pos.x", 0.0f},
    {"cs.ts.pos.y", 0.0f},
    {"cs.ts.pos.z", 0.0f},
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
    // Tool supply servo positions
    {"ts.servo.closems", 1.3f},
    {"ts.servo.openms", 1.6f},
};

#define SETTINGS_COUNT (sizeof(settings) / sizeof(settings[0]))

// Dictionary operations
static int find_setting_index(slice_t key) {
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    if (sl_eq_str(key, settings[i].key)) {
      return i;
    }
  }
  return -1;
}

// Motor-specific setting application under "m."
static bool apply_motor(slice_t key, float value) {
  // Parse: {motor_num}.{key}
  slice_t motor_s = sl_split_at(key, '.', &key);
  if (sl_is_empty(motor_s)) {
    // invalid key format.
    return false;
  }

  // Get motor device
  int motor_num;
  if (!sl_parse_int(motor_s, &motor_num)) {
    return false;
  }
  const struct device* motor = motor_get_device(motor_num);
  if (!motor) {
    return false;
  }

  // Apply setting
  int ret = 0;
  if (sl_eq_str(key, "microstep")) {
    ret = tmc_set_microstep(motor, (int)value);
  } else if (sl_eq_str(key, "current")) {
    ret = tmc_set_current(motor, (int)value, (int)value);
  } else if (sl_eq_str(key, "thresh")) {
    bool enable_stall_detection = value >= 0;
    motor_set_stall_detection(motor_num, enable_stall_detection);
    if (enable_stall_detection) {
      ret = tmc_set_stallguard_threshold(motor, (int)value);
    }
  } else if (sl_eq_str(key, "unitsteps")) {
    motion_set_motor_unitsteps(motor_num, value);
    // Also update wirefeed if this is motor6
    if (motor_num == 6) {
      wirefeed_set_unitsteps(value);
    }
    ret = 0;  // Always succeeds
  } else if (sl_eq_str(key, "idlems")) {
    motor_deenergize_after(motor_num, (int)value);
    ret = 0;  // Always succeeds
  } else {
    return false;
  }

  return ret == 0;
}

// Axis-specific setting application under "a."
static bool apply_axis(slice_t key, float value) {
  // Parse: {axis_name}.{key}
  slice_t axis = sl_split_at(key, '.', &key);
  axis_t axis_num;
  if (sl_eq_str(axis, "x")) {
    axis_num = AXIS_X;
  } else if (sl_eq_str(axis, "y")) {
    axis_num = AXIS_Y;
  } else if (sl_eq_str(axis, "z")) {
    axis_num = AXIS_Z;
  } else {
    return false;  // Invalid axis name
  }

  // Parse {subsystem}.{property} (e.g., "home.origin")
  slice_t subsystem = sl_split_at(key, '.', &key);
  if (sl_eq_str(subsystem, "home") != 0) {
    return false;  // Must be "home"
  }

  // Apply setting
  if (sl_eq_str(key, "origin")) {
    motion_set_home_origin(axis_num, value);
    return true;
  } else if (sl_eq_str(key, "side")) {
    motion_set_home_side(axis_num, value);
    return true;
  } else if (sl_eq_str(key, "phase")) {
    gcode_set_home_phase(axis_num, (int)value);
    return true;
  } else if (sl_eq_str(key, "travel")) {
    motion_set_home_travel(axis_num, value);
    return true;
  }
  return false;
}

// Coordinate system-specific setting application under "cs."
static bool apply_cs(slice_t key, float value) {
  // Parse: {coord_system}.pos.{axis}
  slice_t coord_sys = sl_split_at(key, '.', &key);
  coord_system_t cs_type;
  if (sl_eq_str(coord_sys, "g")) {
    cs_type = COORD_SYSTEM_GRINDER;
  } else if (sl_eq_str(coord_sys, "w")) {
    cs_type = COORD_SYSTEM_WORK;
  } else if (sl_eq_str(coord_sys, "ts")) {
    cs_type = COORD_SYSTEM_TOOLSUPPLY;
  } else {
    return false;
  }

  slice_t attrib = sl_split_at(key, '.', &key);
  if (sl_eq_str(attrib, "pos") != 0) {
    return false;
  }

  slice_t axis = key;
  axis_t axis_index;
  if (sl_eq_str(axis, "x")) {
    axis_index = AXIS_X;
  } else if (sl_eq_str(axis, "y")) {
    axis_index = AXIS_Y;
  } else if (sl_eq_str(axis, "z")) {
    axis_index = AXIS_Z;
  } else if (sl_eq_str(axis, "c")) {
    axis_index = AXIS_C;
  } else {
    return false;
  }

  gcode_set_coord_offset(cs_type, axis_index, value);
  return true;
}

// Tool supply-specific setting application under "ts."
static bool apply_ts(slice_t key, float value) {
  // Parse: servo.{openms|closems}
  slice_t subsys = sl_split_at(key, '.', &key);
  if (!sl_eq_str(subsys, "servo")) {
    return false;
  }

  if (sl_eq_str(key, "openms")) {
    configure_tool_supply_servo_on(TOOL_SUPPLY_OPEN, value);
    return true;
  } else if (sl_eq_str(key, "closems")) {
    configure_tool_supply_servo_on(TOOL_SUPPLY_CLOSED, value);
    return true;
  }

  return false;
}

// Hierarchical apply dispatcher
static bool apply_setting(slice_t key, float value) {
  slice_t category = sl_split_at(key, '.', &key);
  if (sl_eq_str(category, "m")) {
    return apply_motor(key, value);
  } else if (sl_eq_str(category, "a")) {
    return apply_axis(key, value);
  } else if (sl_eq_str(category, "cs")) {
    return apply_cs(key, value);
  } else if (sl_eq_str(category, "ts")) {
    return apply_ts(key, value);
  }
  return false;
}

// Public API
bool settings_set(slice_t key, float value) {
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

float settings_get(slice_t key) {
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

bool settings_apply_all() {
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    bool res =
        apply_setting(sl_from_str((char*)settings[i].key), settings[i].value);
    if (!res) {
      // CM:comm_ps_old_kv_bool("settings.ok", false);
      // CM:comm_ps_old_kv_str("settings.msg", "failed to apply %s",
      // settings[i].key);
      return false;
    }
  }
  // CM:comm_ps_old_kv_bool("settings.ok", true);
  return true;
}
