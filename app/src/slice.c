// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "slice.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

slice_t sl_empty() {
  return (slice_t){.size = 0, .ptr = NULL};
}

slice_t sl_from_buf(uint8_t* buf, int size) {
  return (slice_t){.size = size, .ptr = buf};
}

slice_t sl_from_str(char* str) {
  return (slice_t){.size = (int)strlen(str), .ptr = (uint8_t*)str};
}

bool sl_is_empty(slice_t s) {
  return s.size == 0;
}

slice_t sl_copy(slice_t s, uint8_t* buffer, int size) {
  if (size <= 0) {
    return sl_empty();
  }
  int res_size = s.size < size ? s.size : size;
  memcpy(buffer, s.ptr, res_size);
  return (slice_t){.size = res_size, .ptr = buffer};
}

bool sl_eq(slice_t s1, slice_t s2) {
  if (s1.size != s2.size) {
    return false;
  }
  return memcmp(s1.ptr, s2.ptr, s1.size) == 0;
}

bool sl_eq_str(slice_t s1, const char* s2) {
  return sl_eq(s1, sl_from_str((char*)s2));
}

slice_t sl_sub(slice_t s, int begin, int end) {
  if (begin < 0) {
    begin = 0;
  }
  if (end > s.size) {
    end = s.size;
  }
  if (begin >= end) {
    return sl_empty();
  }
  return (slice_t){.size = end - begin, .ptr = s.ptr + begin};
}

int sl_find(slice_t s, bool (*pred)(uint8_t b, void* ctx), void* pred_ctx) {
  for (int i = 0; i < s.size; i++) {
    if (pred(s.ptr[i], pred_ctx)) {
      return i;
    }
  }
  return -1;
}

static bool pred_is_delim(uint8_t b, void* ctx) {
  char delim = *(char*)ctx;
  return b == (uint8_t)delim;
}

slice_t sl_split_at(slice_t s, char delim, slice_t* remaining) {
  int ix_delim = sl_find(s, pred_is_delim, &delim);
  if (ix_delim < 0) {
    if (remaining) {
      *remaining = sl_empty();
    }
    return s;
  } else {
    if (remaining) {
      *remaining = sl_sub(s, ix_delim + 1, s.size);
    }
    return sl_sub(s, 0, ix_delim);
  }
}

static bool pred_is_space(uint8_t b, void* ctx) {
  return isspace((char)b);
}

static bool pred_is_not_space(uint8_t b, void* ctx) {
  return !isspace((char)b);
}

slice_t sl_split_by_spaces(slice_t s, slice_t* remaining) {
  int ix_space = sl_find(s, pred_is_space, NULL);
  if (ix_space < 0) {
    if (remaining) {
      *remaining = sl_empty();
    }
    return s;
  }

  int ix_next = sl_find(sl_sub(s, ix_space, s.size), pred_is_not_space, NULL);
  if (ix_next < 0) {
    if (remaining) {
      *remaining = sl_empty();
    }
    return sl_sub(s, 0, ix_space);
  }

  if (remaining) {
    *remaining = sl_sub(s, ix_space + ix_next, s.size);
  }
  return sl_sub(s, 0, ix_space);
}

bool sl_parse_int(slice_t s, int* value) {
  // longest int string: INT_MIN "-2147483648" (11 chars)
  if (s.size > 11) {
    return false;
  }
  if (sl_find(s, pred_is_space, NULL) >= 0) {
    return false;  // reject spaces explictly (strtol allows them)
  }

  char buf[12];
  memcpy(buf, s.ptr, s.size);
  buf[s.size] = '\0';

  char* endptr;
  errno = 0;
  long result = strtol(buf, &endptr, 10);
  if (errno != 0) {
    return false;
  }
  if (endptr == buf || *endptr != '\0') {
    return false;
  }
  if (result < INT_MIN || result > INT_MAX) {
    return false;
  }
  *value = (int)result;
  return true;
}

bool sl_parse_float(slice_t s, float* value) {
  // somewhat arbitrary; technically, float can be infinitely long and still
  // valid.
  if (s.size > 20) {
    return false;
  }
  if (sl_find(s, pred_is_space, NULL) >= 0) {
    return false;  // reject spaces explictly (strtof allows them)
  }

  char buf[21];
  memcpy(buf, s.ptr, s.size);
  buf[s.size] = '\0';

  char* endptr;
  errno = 0;
  float result = strtof(buf, &endptr);
  if (errno != 0) {
    return false;
  }
  if (endptr == buf || *endptr != '\0') {
    return false;
  }
  if (result == HUGE_VALF || result == -HUGE_VALF) {
    return false;  // overflow
  }
  if (result != result) {
    return false;  // NaN
  }
  *value = result;
  return true;
}
