// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "strutil.h"

#include <zephyr/ztest.h>

ZTEST(strutil, test_sl_is_empty) {
  zassert_true(sl_is_empty(sl_empty()));
  zassert_true(sl_is_empty(sl_from_str("")));
  zassert_false(sl_is_empty(sl_from_str("abc")));
}

ZTEST(strutil, test_sl_eq) {
  zassert_true(sl_eq(sl_empty(), sl_empty()));
  zassert_true(sl_eq(sl_from_str("abc"), sl_from_str("abc")));

  {
    uint8_t buf1[1];
    uint8_t buf2[1];
    buf1[0] = 'a';
    buf2[0] = 'a';
    slice_t s1 = {.size = 1, .ptr = buf1};
    slice_t s2 = {.size = 1, .ptr = buf2};
    zassert_true(sl_eq(s1, s2), "different pointer, same content");
  }

  {
    uint8_t buf1[2];
    uint8_t buf2[2];
    slice_t s1 = {.size = 2, .ptr = buf1};
    slice_t s2 = {.size = 2, .ptr = buf2};
    buf1[0] = 0;
    buf1[1] = 'a';
    buf2[0] = 0;
    buf2[1] = 'a';
    zassert_true(sl_eq(s1, s2), "content after null matters");

    buf1[0] = 0;
    buf1[1] = 'a';
    buf2[0] = 0;
    buf2[1] = 'b';
    zassert_false(sl_eq(s1, s2), "content after null matters");
  }
}

ZTEST(strutil, test_sl_eq_str) {
  zassert_true(sl_eq_str(sl_empty(), ""));
  zassert_true(sl_eq_str(sl_from_str("abc"), "abc"));

  zassert_false(sl_eq_str(sl_empty(), "a"));
  zassert_false(sl_eq_str(sl_from_str("a"), ""));
}

ZTEST(strutil, test_sl_sub) {
  slice_t s = sl_from_str("abc");

  // basic use cases
  zassert_true(sl_eq_str(sl_sub(s, 0, 3), "abc"));
  zassert_true(sl_eq_str(sl_sub(s, 0, 2), "ab"));
  zassert_true(sl_eq_str(sl_sub(s, 1, 3), "bc"));
  zassert_true(sl_eq_str(sl_sub(s, 0, 0), ""));

  // edge cases
  zassert_true(sl_eq_str(sl_sub(s, 0, 10), "abc"));
  zassert_true(sl_eq_str(sl_sub(s, -10, 3), "abc"));
  zassert_true(sl_eq_str(sl_sub(s, 3, 0), ""));
}

ZTEST(strutil, test_sl_split_at) {
  slice_t ret, rem;

  ret = sl_split_at(sl_from_str("a.b.c"), '.', &rem);
  zassert_true(sl_eq_str(ret, "a"));
  zassert_true(sl_eq_str(rem, "b.c"));

  ret = sl_split_at(sl_from_str("a."), '.', &rem);
  zassert_true(sl_eq_str(ret, "a"));
  zassert_true(sl_is_empty(rem));

  ret = sl_split_at(sl_from_str("a"), '.', &rem);
  zassert_true(sl_eq_str(ret, "a"));
  zassert_true(sl_is_empty(rem));

  ret = sl_split_at(sl_from_str(".a"), '.', &rem);
  zassert_true(sl_is_empty(ret));
  zassert_true(sl_eq_str(rem, "a"));

  ret = sl_split_at(sl_from_str("."), '.', &rem);
  zassert_true(sl_is_empty(ret));
  zassert_true(sl_is_empty(rem));

  ret = sl_split_at(sl_from_str(""), '.', &rem);
  zassert_true(sl_is_empty(ret));
  zassert_true(sl_is_empty(rem));
}

ZTEST(strutil, test_sl_split_at_null_rem) {
  slice_t ret = sl_split_at(sl_from_str("a.b.c"), '.', NULL);
  zassert_true(sl_eq_str(ret, "a"));
}

ZTEST(strutil, test_sl_split_by_spaces) {
  slice_t ret, rem;

  ret = sl_split_by_spaces(sl_from_str("a  b  c"), &rem);
  zassert_true(sl_eq_str(ret, "a"));
  zassert_true(sl_eq_str(rem, "b  c"));

  ret = sl_split_by_spaces(sl_from_str("a "), &rem);
  zassert_true(sl_eq_str(ret, "a"));
  zassert_true(sl_is_empty(rem));

  ret = sl_split_by_spaces(sl_from_str("a"), &rem);
  zassert_true(sl_eq_str(ret, "a"));
  zassert_true(sl_is_empty(rem));

  ret = sl_split_by_spaces(sl_from_str(" a"), &rem);
  zassert_true(sl_is_empty(ret));
  zassert_true(sl_eq_str(rem, "a"));

  ret = sl_split_by_spaces(sl_from_str("  "), &rem);
  zassert_true(sl_is_empty(ret));
  zassert_true(sl_is_empty(rem));

  ret = sl_split_by_spaces(sl_from_str(""), &rem);
  zassert_true(sl_is_empty(ret));
  zassert_true(sl_is_empty(rem));
}

ZTEST(strutil, test_sl_split_by_spaces_null_rem) {
  slice_t ret;

  ret = sl_split_by_spaces(sl_from_str("a  b  c"), NULL);
  zassert_true(sl_eq_str(ret, "a"));

  ret = sl_split_by_spaces(sl_from_str("a "), NULL);
  zassert_true(sl_eq_str(ret, "a"));

  ret = sl_split_by_spaces(sl_from_str("a"), NULL);
  zassert_true(sl_eq_str(ret, "a"));

  ret = sl_split_by_spaces(sl_from_str(" a"), NULL);
  zassert_true(sl_is_empty(ret));

  ret = sl_split_by_spaces(sl_from_str("  "), NULL);
  zassert_true(sl_is_empty(ret));

  ret = sl_split_by_spaces(sl_from_str(""), NULL);
  zassert_true(sl_is_empty(ret));
}

ZTEST(strutil, test_sl_parse_int_valid) {
  int value;
  zassert_true(sl_parse_int(sl_from_str("123"), &value));
  zassert_equal(value, 123);

  zassert_true(sl_parse_int(sl_from_str("-456"), &value));
  zassert_equal(value, -456);

  zassert_true(sl_parse_int(sl_from_str("0"), &value));
  zassert_equal(value, 0);

  zassert_true(sl_parse_int(sl_from_str("2147483647"), &value), "max int32");
  zassert_equal(value, 2147483647);

  zassert_true(sl_parse_int(sl_from_str("-2147483648"), &value), "min int32");
  zassert_equal(value, -2147483648);
}

ZTEST(strutil, test_sl_parse_int_invalid) {
  int value;
  zassert_false(sl_parse_int(sl_from_str(" 123"), &value), "pre-space");
  zassert_false(sl_parse_int(sl_from_str("123 "), &value), "post-space");
  zassert_false(sl_parse_int(sl_from_str("1.2"), &value), "float");
  zassert_false(sl_parse_int(sl_from_str("0x123"), &value), "hex");
  zassert_false(sl_parse_int(sl_from_str("1000000000000000"), &value),
                "bigint");
}

ZTEST(strutil, test_sl_parse_float_valid) {
  float value;
  zassert_true(sl_parse_float(sl_from_str("123.45"), &value));
  zassert_equal(value, 123.45f);

  zassert_true(sl_parse_float(sl_from_str("1.2345e2"), &value));
  zassert_equal(value, 123.45f);
}

ZTEST(strutil, test_sl_parse_float_invalid) {
  float value;
  zassert_false(sl_parse_float(sl_from_str(" 123.45"), &value), "pre-space");
  zassert_false(sl_parse_float(sl_from_str("123.45 "), &value), "post-space");
  zassert_false(sl_parse_float(sl_from_str("12.3.4"), &value), "multi-dot");
  zassert_false(sl_parse_float(sl_from_str("1e1000"), &value), "big float");
  zassert_false(sl_parse_float(sl_from_str("NAN"), &value), "NAN");
  zassert_false(sl_parse_float(sl_from_str("INF"), &value), "INF");
}

ZTEST_SUITE(strutil, NULL, NULL, NULL, NULL, NULL);
