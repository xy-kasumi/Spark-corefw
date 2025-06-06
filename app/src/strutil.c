#include "strutil.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

char* split_at(char* str, char delim) {
  if (!str) {
    return NULL;
  }

  // Find delimiter
  char* p = strchr(str, delim);
  if (p) {
    *p = '\0';     // Null-terminate first part
    return p + 1;  // Return pointer to second part
  }

  // No delimiter found
  return NULL;
}

char* split_by_space(char* str) {
  if (!str) {
    return NULL;
  }

  // Find first whitespace
  char* p = str;
  while (*p && !isspace(*p)) {
    p++;
  }

  if (*p == '\0') {
    return NULL;  // No whitespace found
  }

  *p = '\0';  // Null-terminate first part

  // Skip all consecutive whitespace
  p++;
  while (*p && isspace(*p)) {
    p++;
  }

  return (*p == '\0') ? NULL : p;  // Return NULL if only whitespace remains
}

bool parse_int(const char* str, int* value) {
  if (!str || !value) {
    return false;
  }

  char* endptr;
  errno = 0;
  long result = strtol(str, &endptr, 10);

  // Check for conversion errors
  if (errno != 0) {
    return false;  // Overflow or other error
  }

  // Check if entire string was consumed
  if (endptr == str || *endptr != '\0') {
    return false;  // No digits found or extra characters
  }

  // Check if value fits in int range
  if (result < INT_MIN || result > INT_MAX) {
    return false;  // Out of int range
  }

  *value = (int)result;
  return true;
}

bool parse_float(const char* str, float* value) {
  if (!str || !value) {
    return false;
  }

  char* endptr;
  errno = 0;
  float result = strtof(str, &endptr);

  // Check for conversion errors
  if (errno != 0) {
    return false;  // Overflow or other error
  }

  // Check if entire string was consumed
  if (endptr == str || *endptr != '\0') {
    return false;  // No digits found or extra characters
  }

  *value = result;
  return true;
}
