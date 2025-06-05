#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Parsed G-code command structure
 */
typedef struct {
  uint16_t command;  // G0=0, G1=1, G38.3=383 (×10 for decimals)
  bool has_x, has_y, has_z;
  float x, y, z;
} gcode_parsed_t;

/**
 * Parse a G-code command line
 * @param line Input command line (e.g., "G0 X10.5 Y20.3")
 * @param parsed Output structure to fill with parsed data
 * @return true if parsing succeeded, false on error
 */
bool parse_gcode(const char* line, gcode_parsed_t* parsed);
