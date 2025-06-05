#include "gcode.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// Parse a floating point number from string
static bool parse_float(const char* str, float* result) {
  char* endptr;
  *result = strtof(str, &endptr);
  return endptr != str && (*endptr == '\0' || isspace(*endptr));
}

// Parse G command number, handling decimals like G38.3
static bool parse_g_number(const char* str, uint16_t* command) {
  if (str[0] != 'G') {
    return false;
  }

  str++;  // Skip 'G'

  // Parse integer part
  char* endptr;
  long int_part = strtol(str, &endptr, 10);
  if (endptr == str || int_part < 0 || int_part > 999) {
    return false;
  }

  // Check for decimal part
  if (*endptr == '.') {
    endptr++;  // Skip '.'
    char* dec_endptr;
    long dec_part = strtol(endptr, &dec_endptr, 10);
    if (dec_endptr == endptr || dec_part < 0 || dec_part > 9) {
      return false;
    }
    *command = (uint16_t)(int_part * 10 + dec_part);
    return (*dec_endptr == '\0' || isspace(*dec_endptr));
  } else {
    *command = (uint16_t)(int_part);
    return (*endptr == '\0' || isspace(*endptr));
  }
}

bool parse_gcode(const char* line, gcode_parsed_t* parsed) {
  if (!line || !parsed) {
    return false;
  }

  // Initialize result structure
  *parsed = (gcode_parsed_t){0};

  // Skip leading whitespace
  while (isspace(*line)) {
    line++;
  }

  // Must start with G command
  if (*line != 'G') {
    return false;
  }

  // Parse G command
  const char* token_start = line;
  while (*line && !isspace(*line)) {
    line++;
  }

  // Extract G command token
  size_t token_len = line - token_start;
  char g_token[16];
  if (token_len >= sizeof(g_token)) {
    return false;
  }
  strncpy(g_token, token_start, token_len);
  g_token[token_len] = '\0';

  if (!parse_g_number(g_token, &parsed->command)) {
    return false;
  }

  // Parse optional X, Y, Z parameters
  while (*line) {
    // Skip whitespace
    while (isspace(*line)) {
      line++;
    }

    if (*line == '\0') {
      break;
    }

    // Parse parameter
    char param = *line;
    if (param != 'X' && param != 'Y' && param != 'Z') {
      return false;  // Unknown parameter
    }

    line++;  // Skip parameter letter

    // Find end of value
    token_start = line;
    if (*line == '-' || *line == '+') {
      line++;  // Skip sign
    }
    while (isdigit(*line) || *line == '.') {
      line++;
    }

    if (line == token_start || (line == token_start + 1 &&
                                (*token_start == '-' || *token_start == '+'))) {
      return false;  // No value after parameter
    }

    // Extract value token
    token_len = line - token_start;
    char value_token[32];
    if (token_len >= sizeof(value_token)) {
      return false;
    }
    strncpy(value_token, token_start, token_len);
    value_token[token_len] = '\0';

    // Parse value
    float value;
    if (!parse_float(value_token, &value)) {
      return false;
    }

    // Store in appropriate field
    switch (param) {
      case 'X':
        parsed->has_x = true;
        parsed->x = value;
        break;
      case 'Y':
        parsed->has_y = true;
        parsed->y = value;
        break;
      case 'Z':
        parsed->has_z = true;
        parsed->z = value;
        break;
    }
  }

  return true;
}
