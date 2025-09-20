// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "gcode_base.h"

#include "strutil.h"

#include <string.h>

// Parse G/M command number, handling decimals like G38.3
static bool parse_command_number(slice_t token,
                                 cmd_type_t* cmd_type,
                                 int* code,
                                 int* sub_code) {
  if (sl_is_empty(token)) {
    return false;
  }

  char first_char = token.ptr[0];
  if (first_char == 'G') {
    *cmd_type = CMD_TYPE_G;
  } else if (first_char == 'M') {
    *cmd_type = CMD_TYPE_M;
  } else {
    return false;
  }

  // Parse maincode(.subcode) part.
  slice_t sub;
  slice_t main = sl_split_at(sl_sub(token, 1, token.size), '.', &sub);

  if (!sl_parse_int(main, code) || *code < 0 || *code > 999) {
    return false;
  }
  if (!sl_is_empty(sub)) {
    if (!sl_parse_int(sub, sub_code) || *sub_code < 0 || *sub_code > 9) {
      return false;
    }
  } else {
    *sub_code = -1;
  }
  return true;
}

// Parse axis parameter like "X123" or "X"
static bool parse_axis_param(slice_t token,
                             char expected_axis,
                             axis_state_t* state,
                             float* value) {
  if (sl_is_empty(token)) {
    return false;
  }

  if (token.ptr[0] != expected_axis) {
    return false;
  }

  if (token.size == 1) {
    // Axis only (e.g., "X" for G28 X)
    *state = AXIS_ONLY;
    return true;
  } else {
    // Axis with value (e.g., "X10.5")
    slice_t s_value = sl_sub(token, 1, token.size);
    if (!sl_parse_float(s_value, value)) {
      return false;
    }
    *state = AXIS_WITH_VALUE;
    return true;
  }
}

// Parse parameter like "P500" or "Q2.5"
static bool parse_param(slice_t token,
                        char expected_param,
                        param_state_t* state,
                        float* value) {
  if (sl_is_empty(token)) {
    return false;
  }

  if (token.ptr[0] != expected_param) {
    return false;
  }

  if (token.size == 1) {
    return false;  // Parameter must have value
  }

  // Parse number
  slice_t s_value = sl_sub(token, 1, token.size);
  if (!sl_parse_float(s_value, value)) {
    return false;
  }
  *state = PARAM_SPECIFIED;
  return true;
}

bool parse_gcode(slice_t block, gcode_parsed_t* parsed) {
  if (sl_is_empty(block)) {
    return false;
  }

  // Initialize result structure
  *parsed = (gcode_parsed_t){0};

  // Split into tokens by whitespace
  slice_t token = sl_split_by_spaces(block, &block);

  // First token must be G or M command
  if (!parse_command_number(token, &parsed->cmd_type, &parsed->code,
                            &parsed->sub_code)) {
    return false;
  }

  // Parse remaining parameters
  while (!sl_is_empty(block)) {
    slice_t token = sl_split_by_spaces(block, &block);
    // assert(!sl_is_empty(token));
    char param = token.ptr[0];

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
    } else if (param == 'C') {
      if (!parse_axis_param(token, 'C', &parsed->c_state, &parsed->c)) {
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
