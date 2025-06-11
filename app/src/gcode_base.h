// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Stateless) G-code parsing utilities.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Axis parameter state
 */
typedef enum {
  AXIS_NOT_SPECIFIED,  // Axis not present in command
  AXIS_ONLY,           // Axis present without value (e.g., "X" in G28 X)
  AXIS_WITH_VALUE,     // Axis present with value (e.g., "X10" in G0 X10)
} axis_state_t;

/**
 * Command type
 */
typedef enum {
  CMD_TYPE_NONE,
  CMD_TYPE_G,
  CMD_TYPE_M,
} cmd_type_t;

/**
 * Parameter state
 */
typedef enum {
  PARAM_NOT_SPECIFIED,
  PARAM_SPECIFIED,
} param_state_t;

/**
 * Parsed G-code command structure
 */
typedef struct {
  cmd_type_t cmd_type;  // G-code or M-code
  int code;             // Major number: 0 for G0/M3, 38 for G38.2
  int sub_code;         // Minor number: 2 for G38.2, -1 if not specified

  // Axis parameters (G-codes)
  axis_state_t x_state, y_state, z_state;
  float x, y, z;

  // M-code parameters
  param_state_t p_state, q_state, r_state;
  float p, q, r;
} gcode_parsed_t;

/**
 * Parse a G-code or M-code command line
 * @param line Input command line (e.g., "G0 X10.5 Y20.3" or "M3 P500 Q2.5")
 * @param parsed Output structure to fill with parsed data
 * @return true if parsing succeeded, false on error
 */
bool parse_gcode(const char* line, gcode_parsed_t* parsed);
