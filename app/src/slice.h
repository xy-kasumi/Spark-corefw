// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Stateless) String parsing utilities.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Slice is a view into a byte array.
 * slice_t merely borrows data; it is user's responsibility to ensure data is
 * available while it's used by slice_t.
 *
 * Slice methods do not modify arguments, unless explicitly stated.
 * Size is always >= 0.
 * When size > 0, ptr is never NULL. When size == 0, ptr is undefined.
 */
typedef struct {
  int size;
  uint8_t* ptr;
} slice_t;

/**
 * Create empty slice.
 */
slice_t sl_empty();

/**
 * Create slice from buffer.
 */
slice_t sl_from_buf(uint8_t* buf, int size);

/**
 * Create slice from c-string.
 */
slice_t sl_from_str(char* str);

/**
 * Check if slice is empty.
 */
bool sl_is_empty(slice_t s);

/**
 * Copy slice into a new buffer, and returns slice pointing to it.
 * If buffer is too small, result will be truncated.
 */
slice_t sl_copy(slice_t s, uint8_t* buffer, int size);

/**
 * Compare two slices for equality.
 * @return true if equal.
 */
bool sl_eq(slice_t s1, slice_t s2);

/**
 * Compare slice with c-string for equality.
 * sl_eq_str(s1, s2) == sl_eq(s1, sl_from_str(s2)).
 *
 * @return true if equal.
 */
bool sl_eq_str(slice_t s1, const char* s2);

/**
 * Return sub-slice of s.
 * If begin >= end, returns empty slice.
 *
 * @param begin inclusive. If < 0, treated as 0.
 * @param end exclusive. If > size, treated as size.
 */
slice_t sl_sub(slice_t s, int begin, int end);

/**
 * Split string by first appearance of delim.
 *
 * Examples:
 *   sl_split_at("a.b.c", '.') -> ret="a", rem="b.c"
 *   sl_split_at("a.", '.') -> ret="a", rem=""
 *   sl_split_at("a", '.') -> ret="a", rem=""
 *   sl_split_at(".a", '.') -> ret="", rem="a"
 *   sl_split_at("", '.') -> ret="", rem=""
 *
 * @param remaining (optional) Parts after delim. Empty if no delim found. OK to
 * specify &s.
 * @return Part before delim. Entire s if no delim found.
 */
slice_t sl_split_at(slice_t s, char delim, slice_t* remaining);

/**
 * Split string by first appearance of spaces.
 *
 * Examples:
 *   sl_split_by_spaces("a b c") -> ret="a", rem="b c"
 *   sl_split_by_spaces("a  b") -> str="a", rem="b"
 *   sl_split_by_spaces("a ") -> str="a", rem=""
 *   sl_split_by_spaces("a") -> str="a", rem=""
 *   sl_split_by_spaces("  a") -> ret="", rem="a"
 *   sl_split_by_spaces(" ") -> ret="", retm=""
 *   sl_split_by_spaces("") -> ret="", rem=""
 * @param remaining (optional) TBD
 */
slice_t sl_split_by_spaces(slice_t s, slice_t* remaining);

/**
 * Find index of first byte matching predicate.
 * @param pred return true if match.
 * @param pred_ctx context pointer passed to pred.
 * @return Index of first matching character, or -1 if none found.
 */
int sl_find(slice_t s, bool (*pred)(uint8_t b, void* ctx), void* pred_ctx);

/**
 * Parse int32 from entire s. (base 10)
 * Too big/small value results in failure.
 *
 * @param s input string
 * @param value output value
 * @return true if successful.
 */
bool sl_parse_int(slice_t s, int* value);

/**
 * Parse float32 from entire s.
 * Overflow, inf, nans will result in failure.
 */
bool sl_parse_float(slice_t s, float* value);
