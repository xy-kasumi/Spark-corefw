#include "strutil.h"

#include <string.h>

char* split_front(char** str, char delim) {
  if (!str || !*str) {
    return NULL;
  }

  char* current = *str;

  // Skip leading delimiters
  while (*current == delim) {
    current++;
  }
  if (*current == '\0') {
    *str = NULL;
    return NULL;
  }

  // Start of token
  char* token_start = current;

  // Find delimiter
  char* p = strchr(current, delim);
  if (p) {
    *p = '\0';  // Terminate token
    p++;
    // Skip delimiters after split
    while (*p == delim) {
      p++;
    }
    *str = (*p == '\0') ? NULL : p;
  } else {
    // No delimiter found - this is the last token
    *str = NULL;
  }

  return token_start;
}