// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * Application entry point for spark core firmware.
 * Main command loop is executed here.
 */
#include "canceler.h"
#include "comm.h"
#include "coords.h"
#include "gcode.h"
#include "motion.h"
#include "motor.h"
#include "pulser.h"
#include "settings.h"
#include "slice.h"
#include "toolsupply.h"
#include "wirefeed.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

static bool command_running = false;

// Command: gcode
static void cmd_gcode(slice_t full_command, slice_t maybe_next_command) {
  exec_gcode(full_command, maybe_next_command);
}

// Command: set <key> <value>
static void cmd_set(slice_t command, slice_t args) {
  if (sl_is_empty(args)) {
    comm_error(command, "missing args");
    return;
  }

  slice_t value;
  slice_t key = sl_split_by_spaces(args, &value);
  if (sl_is_empty(value)) {
    comm_error(command, "missing value");
    return;
  }

  // Parse and validate float value
  float float_value;
  if (!sl_parse_float(value, &float_value)) {
    comm_error(command, "invalid float value");
    return;
  }
  if (!settings_set(key, float_value)) {
    comm_error(command, "setting failed");
  }
}

// Command: get
static void cmd_get(slice_t command, slice_t args) {
  if (!sl_is_empty(args)) {
    comm_error(command, "extra args");
    return;
  }

  // List all settings
  const char* key;
  float value;
  comm_ps_begin(PS_SETTINGS);
  for (int i = 0; settings_get_by_index(i, &key, &value); i++) {
    comm_ps_k_vfloat(PS_SETTINGS, key, value);
  }
  comm_ps_end(PS_SETTINGS);
}

// Command: stat
static void cmd_stat(slice_t command, slice_t args) {
  if (!sl_is_empty(args)) {
    comm_error(command, "extra args");
    return;
  }

  comm_ps_begin(PS_STAT);
  motor_dump_status();
  if (canceler_cancel_needed()) {
    return;
  }
  pulser_dump_status();
  if (canceler_cancel_needed()) {
    return;
  }
  wirefeed_dump_status();
  if (canceler_cancel_needed()) {
    return;
  }
  comm_ps_end(PS_STAT);
}

// Command: test
static void cmd_test(slice_t command, slice_t args) {
  if (sl_is_empty(args)) {
    comm_error(command, "missing args");
    return;
  }

  // Split target from rest of args
  slice_t params;
  slice_t target = sl_split_at(args, ' ', &params);

  if (sl_eq_str(target, "pulser")) {
    // Parse duration parameter
    if (sl_is_empty(params)) {
      comm_error(command, "missing duration[sec]");
      return;
    }

    int duration;
    if (!sl_parse_int(params, &duration) || duration <= 0) {
      comm_error(command, "invalid duration");
      return;
    }

    exec_test_pulser(duration);
  } else {
    comm_error(command, "unknown target");
  }
}

static bool is_gcode(slice_t command) {
  return command.size > 0 && (command.ptr[0] == 'G' || command.ptr[0] == 'M');
}

/**
 * @param next_command empty if not available.
 */
static void handle_command(slice_t command, slice_t next_command) {
  // G-code or command?
  if (is_gcode(command)) {
    bool next_avail = !sl_is_empty(next_command) && is_gcode(next_command);
    cmd_gcode(command, next_avail ? next_command : sl_empty());
  } else {
    slice_t args;
    slice_t cmd = sl_split_by_spaces(command, &args);

    // Dispatch to command handler
    if (sl_eq_str(cmd, "stat")) {
      cmd_stat(command, args);
    } else if (sl_eq_str(cmd, "set")) {
      cmd_set(command, args);
    } else if (sl_eq_str(cmd, "get")) {
      cmd_get(command, args);
    } else if (sl_eq_str(cmd, "test")) {
      cmd_test(command, args);
    } else {
      comm_error(command, "unknown command");
    }
  }
}

static void handle_signal(payload_t* payload) {
  slice_t signal = sl_from_buf(payload->data, payload->size);
  if (sl_eq_str(signal, "!")) {
    canceler_cancel();
  } else if (sl_eq_str(signal, "?pos")) {
    // Print ready with current position
    pos_phys_t machine_pos = motion_get_current_pos();

    // TODO: these are not thread-safe.
    coord_system_t current_cs = gcode_get_current_coord_system();
    const coord_offsets_t* offsets = gcode_get_coord_offsets();

    const char* cs_name = "";
    const char* prefix = "";
    switch (current_cs) {
      case COORD_SYSTEM_GRINDER:
        cs_name = "grinder";
        prefix = "g";
        break;
      case COORD_SYSTEM_WORK:
        cs_name = "work";
        prefix = "w";
        break;
      case COORD_SYSTEM_TOOLSUPPLY:
        cs_name = "toolsupply";
        prefix = "t";
        break;
      case COORD_SYSTEM_MACHINE:
        cs_name = "machine";
        prefix = "m";
        break;
    }

    comm_ps_raw(PS_POS, "< sys:\"%s\" m.x:%.3f m.y:%.3f m.z:%.3f m.c:%.3f%s",
                cs_name, (double)machine_pos.x, (double)machine_pos.y,
                (double)machine_pos.z, (double)(machine_pos.c * 360.0f),
                (current_cs == COORD_SYSTEM_MACHINE ? " >" : ""));
    if (current_cs == COORD_SYSTEM_MACHINE) {
      return;
    }

    pos_phys_t cs_pos = coords_from_machine(&machine_pos, current_cs, offsets);
    comm_ps_raw(PS_POS, "%s.x:%.3f %s.y:%.3f %s.z:%.3f %s.c:%.3f >", prefix,
                (double)cs_pos.x, prefix, (double)cs_pos.y, prefix,
                (double)cs_pos.z, prefix, (double)(cs_pos.c * 360.0f));
  } else if (sl_eq_str(signal, "?queue")) {
    int cap;
    int used;
    comm_stat_command_queue(&cap, &used);
    int num_executing = command_running ? 1 : 0;
    comm_ps_raw(PS_QUEUE, "< cap:%d num:%d >", cap, used + num_executing);
  } else if (sl_eq_str(signal, "?edm")) {
    ps_edm_t edm = motion_get_edm_state();
    comm_ps_begin(PS_EDM);
    if (edm.has_edm_data) {
      comm_ps_raw(PS_EDM, "open:%.1f short:%.1f", (double)edm.r_open,
                  (double)edm.r_short);
    }
    if (edm.is_moving) {
      comm_ps_raw(PS_EDM, "pb_f:%.3f pb_b:%.3f dist:%.3f dist_max:%.3f",
                  (double)edm.pb_front, (double)edm.pb_back,
                  (double)edm.distance, (double)edm.distance_max);
    }
    comm_ps_end(PS_EDM);
  } else {
    // Unknown signal. Probably better to ignore, to not clog stream.
  }
}

int main() {
  k_float_enable(k_work_queue_thread_get(&k_sys_work_q), K_FP_REGS);
  k_float_enable(k_current_get(), K_FP_REGS);

  // init core
  canceler_init();
  comm_init(handle_signal);

  comm_ps_begin(PS_INIT);

  // init hardware
  bool ok = true;
  ok &= motor_init();
  ok &= pulser_init();
  ok &= toolsupply_init();
  if (!ok) {
    // cannot proceed to module init if hardware is failing
    comm_ps_k_vbool(PS_INIT, "ok", false);
    comm_ps_end(PS_INIT);
    return 1;
  }

  // init modules
  ok &= motion_init();
  ok &= wirefeed_init();
  if (!ok) {
    comm_ps_k_vbool(PS_INIT, "ok", false);
    comm_ps_end(PS_INIT);
    return 1;
  }

  // apply default settings
  ok &= settings_apply_all();

  comm_ps_k_vbool(PS_INIT, "ok", ok);
  comm_ps_end(PS_INIT);

  while (true) {
    // Ignore commands during cancelation window.
    while (canceler_cancel_needed()) {
      comm_clear_commands();
      k_sleep(K_MSEC(50));
    }

    comm_wait_for_command();

    // Long enough for queue to accumulate,
    // but short enough for user to feel responsive.
    k_sleep(K_MSEC(100));

    // Keep executing without wait until queue is fully consumed.
    while (!canceler_cancel_needed()) {
      payload_t cmd;
      payload_t next_cmd;
      int num_avail = comm_get_command_if_avail(&cmd, &next_cmd);
      if (num_avail == 0) {
        // comm_clear_commands() was called or queue exhausted
        break;
      }
      command_running = true;
      handle_command(sl_from_buf(cmd.data, cmd.size),
                     num_avail >= 2 ? sl_from_buf(next_cmd.data, next_cmd.size)
                                    : sl_empty());
      command_running = false;
    }
  }

  return 0;
}
