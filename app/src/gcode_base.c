// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "gcode_base.h"

#include "strutil.h"

#include <string.h>

// Parse G/M command number, handling decimals like G38.3
static bool parse_command_number(char* token,
                                 cmd_type_t* cmd_type,
                                 int* code,
                                 int* sub_code) {
  if (token[0] == 'G') {
    *cmd_type = CMD_TYPE_G;
  } else if (token[0] == 'M') {
    *cmd_type = CMD_TYPE_M;
  } else {
    return false;
  }

  // Skip command letter and parse main code
  char* code_part = token + 1;
  char* sub_part = split_at(code_part, '.');

  if (!parse_int(code_part, code) || *code < 0 || *code > 999) {
    return false;
  }

  if (sub_part) {
    if (!parse_int(sub_part, sub_code) || *sub_code < 0 || *sub_code > 9) {
      return false;
    }
  } else {
    *sub_code = -1;
  }

  return true;
}

// Parse axis parameter like "X123" or "X"
static bool parse_axis_param(const char* token,
                             char expected_axis,
                             axis_state_t* state,
                             float* value) {
  if (token[0] != expected_axis) {
    return false;
  }

  if (strlen(token) == 1) {
    // Axis only (e.g., "X" for G28 X)
    *state = AXIS_ONLY;
    return true;
  } else {
    // Axis with value (e.g., "X10.5")
    const char* value_str = token + 1;  // Skip axis letter
    if (!parse_float(value_str, value)) {
      return false;
    }
    *state = AXIS_WITH_VALUE;
    return true;
  }
}

// Parse parameter like "P500" or "Q2.5"
static bool parse_param(const char* token,
                        char expected_param,
                        param_state_t* state,
                        float* value) {
  if (token[0] != expected_param) {
    return false;
  }

  if (strlen(token) == 1) {
    return false;  // Parameter must have value
  }

  const char* value_str = token + 1;  // Skip parameter letter
  if (!parse_float(value_str, value)) {
    return false;
  }

  *state = PARAM_SPECIFIED;
  return true;
}

bool parse_gcode(const char* line, gcode_parsed_t* parsed) {
  if (!line || !parsed) {
    return false;
  }

  // Initialize result structure
  *parsed = (gcode_parsed_t){0};

  // Make mutable copy for parsing
  char mut_line[256];
  strncpy(mut_line, line, sizeof(mut_line) - 1);
  mut_line[sizeof(mut_line) - 1] = '\0';

  // Split into tokens by whitespace
  char* token = mut_line;
  char* rest = split_by_space(token);

  // First token must be G or M command
  if (!parse_command_number(token, &parsed->cmd_type, &parsed->code,
                            &parsed->sub_code)) {
    return false;
  }

  // Parse remaining parameters
  while (rest) {
    token = rest;
    rest = split_by_space(token);

    char param = token[0];

    // Try axis parameters (for G-codes)
    if (param == 'X') {
      if (!parse_axis_param(token, 'X', &parsed->x_state, &parsed->x)) {
        return false;
      }
    } else if (param == 'Y') {
      if (!parse_axis_param(token, 'Y', &parsed->y_state, &parsed->y)) {
        return false;
      }
    } else if (param == 'Z') {
      if (!parse_axis_param(token, 'Z', &parsed->z_state, &parsed->z)) {
        return false;
      }
    }
    // Try P/Q/R parameters (for M-codes)
    else if (param == 'P') {
      if (!parse_param(token, 'P', &parsed->p_state, &parsed->p)) {
        return false;
      }
    } else if (param == 'Q') {
      if (!parse_param(token, 'Q', &parsed->q_state, &parsed->q)) {
        return false;
      }
    } else if (param == 'R') {
      if (!parse_param(token, 'R', &parsed->r_state, &parsed->r)) {
        return false;
      }
    } else {
      return false;  // Unknown parameter
    }
  }

  return true;
}
