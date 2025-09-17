// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * Application entry point for spark core firmware.
 * Main command loop is executed here.
 */
#include "comm.h"
#include "coords.h"
#include "gcode.h"
#include "motion.h"
#include "motor.h"
#include "pulser.h"
#include "settings.h"
#include "strutil.h"
#include "system.h"
#include "toolsupply.h"
#include "wirefeed.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

static uint8_t download_buffer[40000];
static uint32_t download_buffer_size = 0;

// Command: gcode
static void cmd_gcode(char* full_command) {
  exec_gcode(full_command);
}

// Command: set <key> <value>
static void cmd_set(char* args) {
  if (!args) {
    // CM:comm_print_err("Usage: set <key> <value>");
    return;
  }

  // Destructive parse: split key and value
  char* key = args;
  char* value = split_at(key, ' ');

  if (!value) {
    // CM:comm_print_err("Usage: set <key> <value>");
    return;
  }

  // Parse and validate float value
  float float_value;
  if (!parse_float(value, &float_value)) {
    // CM:comm_print_err("Invalid number: %s", value);
    return;
  }

  if (!settings_set(key, float_value)) {
    // CM:comm_print_err("Failed to set %s", key);
  }
}

// Command: get
static void cmd_get(char* args) {
  if (args && strlen(args) > 0) {
    // CM:comm_print_err("Usage: get");
    return;
  }

  // List all settings
  const char* key;
  float value;
  comm_print_noprefix("settings <");
  for (int i = 0; settings_get_by_index(i, &key, &value); i++) {
    comm_print_noprefix("settings %s:%.1f", key, (double)value);
  }
  comm_print_noprefix("settings >");
}

// Command: stat
static void cmd_stat(char* args) {
  if (!args || strlen(args) == 0) {
    comm_print_err("Usage: stat <subsystem>");
    // CM:comm_print("Available subsystems: motor, pulser, wirefeed");
    return;
  }

  if (strcmp(args, "motor") == 0) {
    motor_dump_status();
  } else if (strcmp(args, "pulser") == 0) {
    pulser_dump_status();
  } else if (strcmp(args, "wirefeed") == 0) {
    wirefeed_dump_status();
  } else {
    comm_print_err("Unknown subsystem: %s", args);
  }
}

// Command: download
static void cmd_download(char* args) {
  // Copy EDM log data to download buffer
  download_buffer_size =
      pulser_copy_log_to_buffer(download_buffer, sizeof(download_buffer));

  if (download_buffer_size == 0) {
    // CM:comm_print("No EDM data available");
    return;
  }

  // uint32_t entry_count = download_buffer_size / 4;  // 4 bytes per entry
  // CM:comm_print("Sending %u bytes (%u EDM entries)",
  // download_buffer_size,entry_count);
  comm_print_blob(download_buffer, download_buffer_size);
}

// Command: test
static void cmd_test(char* args) {
  if (!args || strlen(args) == 0) {
    // CM:comm_print_err("Usage: test <target> <params...>");
    return;
  }

  // Split target from rest of args
  char* target = args;
  char* params = split_at(target, ' ');

  if (strcmp(target, "pulser") == 0) {
    // Parse duration parameter
    if (!params || strlen(params) == 0) {
      // CM:comm_print_err("Usage: test pulser <duration_sec>");
      return;
    }

    int duration;
    if (!parse_int(params, &duration) || duration <= 0) {
      // CM:comm_print_err("Invalid duration: %s", params);
      return;
    }

    exec_test_pulser(duration);
  } else {
    // CM:comm_print_err("Unknown test target: %s", target);
  }
}

static void handle_console_command(char* command) {
  g_machine_state = STATE_EXEC_INTERACTIVE;
  comm_print_ack();

  // G-code or command?
  if (command[0] == 'G' || command[0] == 'M') {
    cmd_gcode(command);
  } else {
    // Destructive parse: split command and arguments
    char* cmd = command;
    char* args = split_at(cmd, ' ');

    // Dispatch to command handler
    if (strcmp(cmd, "stat") == 0) {
      cmd_stat(args);
    } else if (strcmp(cmd, "set") == 0) {
      cmd_set(args);
    } else if (strcmp(cmd, "get") == 0) {
      cmd_get(args);
    } else if (strcmp(cmd, "download") == 0) {
      cmd_download(args);
    } else if (strcmp(cmd, "test") == 0) {
      cmd_test(args);
    } else if (strcmp(cmd, "ping") == 0) {
      // Do nothing
    } else {
      // CM:comm_print_err("unknown command: %s; type 'help' for available
      // commands",cmd);
    }
  }

  // Clear cancel flag and return to IDLE
  g_cancel_requested = false;
  g_machine_state = STATE_IDLE;

  // Print ready with current position
  pos_phys_t machine_pos = motion_get_current_pos();
  coord_system_t current_cs = gcode_get_current_coord_system();
  const coord_offsets_t* offsets = gcode_get_coord_offsets();

  if (current_cs == COORD_SYSTEM_MACHINE) {
    comm_print("ready X%.3f Y%.3f Z%.3f C%.3f (machine)", (double)machine_pos.x,
               (double)machine_pos.y, (double)machine_pos.z,
               (double)(machine_pos.c * 360.0f));
  } else {
    // Convert machine position to current coordinate system for display
    /*pos_phys_t cs_pos =*/coords_from_machine(&machine_pos, current_cs,
                                               offsets);
    const char* cs_name = "";
    switch (current_cs) {
      case COORD_SYSTEM_GRINDER:
        cs_name = "grinder";
        break;
      case COORD_SYSTEM_WORK:
        cs_name = "work";
        break;
      case COORD_SYSTEM_TOOLSUPPLY:
        cs_name = "toolsupply";
        break;
      case COORD_SYSTEM_MACHINE:
        // shouldn't happen
    }
    // CM:comm_print("ready X%.3f Y%.3f Z%.3f C%.3f (%s) X%.3f Y%.3f Z%.3f C%.3f
    // (machine)",(double)cs_pos.x, (double)cs_pos.y,
    // (double)cs_pos.z,(double)(cs_pos.c * 360.0f), cs_name,
    // (double)machine_pos.x,(double)machine_pos.y,
    // (double)machine_pos.z,(double)(machine_pos.c * 360.0f));
  }
}

void handle_signal(const char* payload) {}

int main() {
  // init core
  state_machine_init();
  comm_init(handle_signal);

  comm_ps_old_begin("init");

  // init hardware
  bool ok = true;
  ok &= motor_init();
  ok &= pulser_init();
  ok &= toolsupply_init();
  if (!ok) {
    // cannot proceed to module init if hardware is failing
    comm_ps_old_kv_bool("ok", false);
    comm_ps_old_end();
    return 1;
  }

  // init modules
  ok &= motion_init();
  ok &= wirefeed_init();
  if (!ok) {
    comm_ps_old_kv_bool("ok", false);
    comm_ps_old_end();
    return 1;
  }

  // apply default settings
  ok &= settings_apply_all();

  comm_ps_old_kv_bool("ok", ok);
  comm_ps_old_end();

  // LEGACY
  comm_print_noprefix("I ready");

  while (1) {
    comm_wait_for_command();
    payload_t cmd;
    payload_t next_cmd;
    int num_avail = comm_get_command_if_avail(&cmd, &next_cmd);
    if (num_avail == 0) {
      // comm_clear_commands() was called in signal handler
      continue;
    }
    handle_console_command(cmd.data);
  }

  return 0;
}
