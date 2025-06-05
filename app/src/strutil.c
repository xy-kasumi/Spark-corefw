#include "strutil.h"

#include <string.h>

char* split_at(char* str, char delim) {
  if (!str) {
    return NULL;
  }

  // Find delimiter
  char* p = strchr(str, delim);
  if (p) {
    *p = '\0';  // Null-terminate first part
    return p + 1;  // Return pointer to second part
  }

  // No delimiter found
  return NULL;
}
