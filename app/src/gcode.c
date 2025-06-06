#include "gcode.h"

#include "strutil.h"

#include <string.h>

// Parse G command number, handling decimals like G38.3
static bool parse_g_number(char* token, int* code, int* sub_code) {
  if (token[0] != 'G') {
    return false;
  }

  // Skip 'G' and parse main code
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

  // First token must be G command
  if (!parse_g_number(token, &parsed->code, &parsed->sub_code)) {
    return false;
  }

  // Parse remaining axis parameters
  while (rest) {
    token = rest;
    rest = split_by_space(token);

    // Try to parse as axis parameter
    char axis = token[0];
    if (axis == 'X') {
      if (!parse_axis_param(token, 'X', &parsed->x_state, &parsed->x)) {
        return false;
      }
    } else if (axis == 'Y') {
      if (!parse_axis_param(token, 'Y', &parsed->y_state, &parsed->y)) {
        return false;
      }
    } else if (axis == 'Z') {
      if (!parse_axis_param(token, 'Z', &parsed->z_state, &parsed->z)) {
        return false;
      }
    } else {
      return false;  // Unknown parameter
    }
  }

  return true;
}
