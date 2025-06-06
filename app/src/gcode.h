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
 * Parsed G-code command structure
 */
typedef struct {
  int code;      // Major number: 0 for G0, 38 for G38.2
  int sub_code;  // Minor number: 2 for G38.2, -1 if not specified
  axis_state_t x_state, y_state, z_state;
  float x, y, z;
} gcode_parsed_t;

/**
 * Parse a G-code command line
 * @param line Input command line (e.g., "G0 X10.5 Y20.3")
 * @param parsed Output structure to fill with parsed data
 * @return true if parsing succeeded, false on error
 */
bool parse_gcode(const char* line, gcode_parsed_t* parsed);
