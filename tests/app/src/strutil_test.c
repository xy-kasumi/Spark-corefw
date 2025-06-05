#include "strutil.h"
#include <zephyr/ztest.h>

ZTEST(strutil, test_split_at) {
  char str[] = "a,b";
  char* rem = split_at(str, ',');

  zassert_not_null(rem, "Split should return non-NULL");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
  zassert_equal(rem[0], 'b', "Second part should be 'b'");
  zassert_equal(rem[1], 0, "Second part should be null-terminated");
}

ZTEST(strutil, test_split_at_dupe) {
  char str[] = "a,,b";
  char* rem = split_at(str, ',');

  zassert_not_null(rem, "Split should return non-NULL");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
  zassert_equal(rem[0], ',', "Second part should be ','");
  zassert_equal(rem[1], 'b', "Second part should be 'b'");
  zassert_equal(rem[2], 0, "Second part should be null-terminated");
}

ZTEST(strutil, test_split_not_found) {
  char str[] = "a";
  char* rem = split_at(str, ',');

  zassert_is_null(rem, "Second part should be NULL.");
  zassert_equal(str[0], 'a', "First part should be 'a'");
  zassert_equal(str[1], 0, "First part should be null-terminated");
}

ZTEST(strutil, test_parse_int_valid) {
  int value;
  zassert_true(parse_int("123", &value), "Should parse valid int");
  zassert_equal(value, 123, "Value should be 123");

  zassert_true(parse_int("-456", &value), "Should parse negative int");
  zassert_equal(value, -456, "Value should be -456");

  zassert_true(parse_int("0", &value), "Should parse zero");
  zassert_equal(value, 0, "Value should be 0");
}

ZTEST(strutil, test_parse_int_invalid) {
  int value;
  zassert_false(parse_int("abc", &value), "Should reject non-numeric");
  zassert_false(parse_int("123x", &value), "Should reject partial numeric");
  zassert_false(parse_int("", &value), "Should reject empty string");
  zassert_false(parse_int(NULL, &value), "Should reject NULL string");
  zassert_false(parse_int("123", NULL), "Should reject NULL output");
}

ZTEST(strutil, test_parse_float_valid) {
  float value;
  zassert_true(parse_float("123.5", &value), "Should parse valid float");
  zassert_within(value, 123.5f, 0.001f, "Value should be 123.5");

  zassert_true(parse_float("-45.67", &value), "Should parse negative float");
  zassert_within(value, -45.67f, 0.001f, "Value should be -45.67");

  zassert_true(parse_float("0.0", &value), "Should parse zero float");
  zassert_within(value, 0.0f, 0.001f, "Value should be 0.0");

  zassert_true(parse_float("42", &value), "Should parse int as float");
  zassert_within(value, 42.0f, 0.001f, "Value should be 42.0");
}

ZTEST(strutil, test_parse_float_invalid) {
  float value;
  zassert_false(parse_float("abc", &value), "Should reject non-numeric");
  zassert_false(parse_float("12.3x", &value), "Should reject partial numeric");
  zassert_false(parse_float("", &value), "Should reject empty string");
  zassert_false(parse_float(NULL, &value), "Should reject NULL string");
  zassert_false(parse_float("12.3", NULL), "Should reject NULL output");
}

ZTEST_SUITE(strutil, NULL, NULL, NULL, NULL, NULL);
